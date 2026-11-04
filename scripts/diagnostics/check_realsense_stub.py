#!/usr/bin/env python3
"""Exercise the RealSense camera code path with a fake pyrealsense2 module (no device)."""
import sys
import types

import numpy as np


def make_fake_rs():
    rs = types.ModuleType("pyrealsense2")

    class Format:
        rgb8 = "rgb8"
        bgr8 = "bgr8"
        y8 = "y8"

    class StreamKind:
        color = "color"
        infrared = "infrared"

    class Frame:
        def __init__(self, data):
            self._data = data

        def get_data(self):
            return self._data

    class Frames:
        def __init__(self, data):
            self._data = data

        def get_color_frame(self):
            return Frame(self._data)

        def get_infrared_frame(self, index=1):
            return Frame(self._data)

    class Pipeline:
        def __init__(self):
            self.running = False
            self.size = None

        def start(self, config):
            if config.size is None:
                raise RuntimeError("no stream enabled")
            width, height = config.size
            if width >= 1280:
                raise RuntimeError("1280x720 unavailable on this fake device")
            self.running = True
            self.size = config.size

        def stop(self):
            self.running = False

        def wait_for_frames(self):
            width, height = self.size
            data = np.zeros((height, width, 3), dtype=np.uint8)
            data[:, :, 0] = 7
            return Frames(data)

    class Config:
        def __init__(self):
            self.size = None

        def enable_device(self, serial):
            self.serial = serial

        def enable_stream(self, stream, *args):
            if stream == rs.stream.color:
                if len(args) == 4:
                    width, height, fmt, fps = args
                else:
                    width, height, fmt, fps = args[1:]
                self.size = (int(width), int(height))

    rs.format = Format
    rs.stream = StreamKind
    rs.pipeline = Pipeline
    rs.config = Config
    return rs


def main():
    sys.modules["pyrealsense2"] = make_fake_rs()
    from roboapi_yam.camera import YamCamera, YamCameraConfig

    cfg = YamCameraConfig(source="realsense", serial="123", width=1280, height=720, fps=30)
    cfg.validate()
    cam = YamCamera(cfg)
    cam.connect()
    frame, age = cam.read()
    print("stream", cam._stream, "format", cam._format, "capture", cam._capture)
    print("frame", frame.shape, frame.dtype, "age_ms", round(age * 1000, 1))
    assert frame.shape == (720, 1280, 3), frame.shape
    after = cam.read()[1]
    nxt = cam.read_after(after, 1.0)
    print("read_after ok", nxt is not None)
    cam.disconnect()
    print("camera fake-path OK")


if __name__ == "__main__":
    main()
