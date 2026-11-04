"""YAM wrist camera: RealSense colour stream, or a synthetic source for hardware-free dry runs."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

from roboapi.hardware import CameraConfig, ComponentOption, ComponentPlugin

SOURCES = ("synthetic", "realsense")


@dataclass
class YamCameraConfig(CameraConfig):
    type: str = "yam"
    source: str = "synthetic"
    serial: str = ""
    width: int = 1280
    height: int = 720
    fps: int = 30

    @property
    def connection_id(self) -> str:
        return f"{self.source}:{self.serial or (str(self.width) + 'x' + str(self.height))}"

    def validate(self) -> None:
        super().validate()
        if self.source not in SOURCES:
            raise ValueError(f"camera.source must be one of {SOURCES}")
        for name in ("width", "height", "fps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"camera.{name} must be a positive integer")


def configure(option, setup):
    source = SOURCES[setup.choose("Camera source", ["synthetic (dry run)", "RealSense UVC"])]
    return option.config_type(source=source, serial=setup.ask("RealSense serial (blank = first)", ""))


def camera_plugin():
    return ComponentPlugin(
        options=(ComponentOption("YAM wrist camera (synthetic)", YamCameraConfig, {"source": "synthetic"}),
                 ComponentOption("YAM wrist camera (RealSense)", YamCameraConfig, {"source": "realsense"})),
        create=YamCamera, configure=configure,
        setup_notes=("Use synthetic while the cameras are busy; RealSense mode needs the device idle.",),
    )


class YamCamera:
    """Latest-frame worker; images are raw-view uint8 RGB, age is measured from capture."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.last_error = None
        self._cond = threading.Condition()
        self._frame = None
        self._stamp = None
        self._stop = threading.Event()
        self._thread = None
        self._pipeline = None
        self._base = None
        self._stream = "color"
        self._format = "rgb8"
        self._capture = (self.cfg.width, self.cfg.height)

    @property
    def live(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and self.last_error is None

    def connect(self) -> None:
        _trace("camera.connect enter")
        if self._thread is not None and self._thread.is_alive():
            return
        if self.cfg.source == "realsense":
            self._open_realsense()
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, name="yam-camera", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 2.0
        with self._cond:
            while self._frame is None and time.monotonic() < deadline:
                self._cond.wait(0.05)
        if self._frame is None:
            self.last_error = self.last_error or "camera produced no frame within 2 s"
            raise RuntimeError(self.last_error)

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
            self._pipeline = None

    def read(self):
        with self._cond:
            frame, stamp = self._frame, self._stamp
        if frame is None:
            raise RuntimeError("camera has no frame yet")
        return frame.copy(), max(0.0, time.monotonic() - stamp)

    def read_after(self, after, timeout_s, should_stop=None):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._cond:
            while True:
                if self._frame is not None and (after is None or self._stamp > after):
                    return self._frame.copy(), max(0.0, time.monotonic() - self._stamp)
                stop = should_stop() if callable(should_stop) else bool(should_stop)
                if stop:
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(min(remaining, 0.05))

    # -- sources -----------------------------------------------------------
    def _open_realsense(self) -> None:
        """Try 720p RGB first, then 480p RGB, then the infrared stream.

        A D405 or another model may not expose rgb8 at 1280x720 while it also streams depth;
        falling back keeps the console usable, and the image is resized (never cropped) to the
        1280x720 contract. The chosen mode is logged by the caller through last_error only when
        every attempt fails.
        """
        import pyrealsense2 as rs

        width, height = int(self.cfg.width), int(self.cfg.height)
        fps = int(self.cfg.fps)
        attempts = [("color", "rgb8", width, height), ("color", "bgr8", width, height),
                    ("color", "rgb8", 640, 480), ("color", "bgr8", 640, 480),
                    ("infrared", "y8", width, height)]
        last = None
        for stream, fmt, w, h in attempts:
            pipeline = rs.pipeline()
            config = rs.config()
            if self.cfg.serial:
                config.enable_device(self.cfg.serial)
            try:
                if stream == "color":
                    config.enable_stream(rs.stream.color, w, h, getattr(rs.format, fmt), fps)
                else:
                    config.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
                pipeline.start(config)
            except Exception as exc:  # noqa: BLE001 - try the next mode
                last = exc
                try:
                    pipeline.stop()
                except Exception:  # noqa: BLE001
                    pass
                continue
            self._pipeline = pipeline
            self._stream = stream
            self._format = fmt
            self._capture = (w, h)
            return
        raise RuntimeError("no usable RealSense stream (" + repr(last) + ")")

    def _grab(self) -> np.ndarray:
        if self.cfg.source == "realsense":
            frames = self._pipeline.wait_for_frames()
            frame = frames.get_color_frame() if self._stream == "color" else frames.get_infrared_frame(1)
            if not frame:
                raise RuntimeError("no " + self._stream + " frame")
            image = np.asanyarray(frame.get_data())
            if self._format == "bgr8":
                image = image[:, :, ::-1]
            elif self._format == "y8":
                image = np.repeat(image[:, :, None], 3, axis=2)
            image = np.ascontiguousarray(image, dtype=np.uint8)
            return self._fit(image)
        return self._synthetic()

    def _fit(self, image: np.ndarray) -> np.ndarray:
        """Resize to the camera contract without cropping or mirroring."""
        height, width = int(self.cfg.height), int(self.cfg.width)
        if image.shape[:2] == (height, width):
            return image
        import cv2

        return cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)

    def _synthetic(self) -> np.ndarray:
        height, width = self.cfg.height, self.cfg.width
        if self._base is None:
            yy, xx = np.mgrid[0:height, 0:width]
            self._base = np.stack([(xx * 255 // width), (yy * 255 // height),
                                   ((xx + yy) * 255 // (width + height))], axis=-1).astype(np.uint8)
        frame = self._base.copy()
        phase = int((time.monotonic() * 120.0) % width)
        frame[:, phase:phase + 48] = np.array([255, 64, 64], dtype=np.uint8)
        frame[8:24, 8:8 + int((time.monotonic() * 40) % 200)] = 255
        return frame

    def _worker(self) -> None:
        period = 1.0 / max(float(self.cfg.fps), 1.0)
        deadline = time.perf_counter()
        while not self._stop.is_set():
            try:
                frame = self._grab()
                stamp = time.monotonic()
                with self._cond:
                    self._frame = frame
                    self._stamp = stamp
                    self._cond.notify_all()
                self.last_error = None
            except Exception as exc:  # noqa: BLE001
                self.last_error = f"{type(exc).__name__}: {exc}"
                time.sleep(0.05)
            deadline += period
            sleep = deadline - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                deadline = time.perf_counter()

def _trace(event):
    import os
    path = os.environ.get("ROBOAPI_YAM_LOG")
    if not path:
        return
    import time
    try:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write("EVENT %s %s\n" % (format(time.monotonic(), '.1f'), event))
    except Exception:
        pass
