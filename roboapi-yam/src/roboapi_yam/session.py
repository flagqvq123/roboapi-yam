"""One robot session per (mode, channel): shared device handle, control loop and FK/IK.

RoboAPI drives the arm through Cartesian targets at 30 Hz. This module owns the only object that
talks to the chain: it keeps a 100 Hz loop that solves IK (warm-started, ~0.4 ms) and streams joint
targets. The arm and gripper components are thin views onto this session, which is what lets them
share a single CAN handle.
"""

from __future__ import annotations

import atexit
import os
import threading
import time

import numpy as np

from .backend import create_backend
from .kin import YamKinematics

_SESSIONS: dict[tuple, "YamSession"] = {}
_SESSIONS_LOCK = threading.Lock()
_KIN_CACHE: dict[str, YamKinematics] = {}
_KIN_LOCK = threading.Lock()


def kinematics_for(arm_type: str) -> YamKinematics:
    with _KIN_LOCK:
        kin = _KIN_CACHE.get(arm_type)
        if kin is None:
            from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

            path = combine_arm_and_gripper_xml(ArmType(arm_type), GripperType.NO_GRIPPER)
            kin = YamKinematics(path)
            _KIN_CACHE[arm_type] = kin
        return kin


def session_key(cfg) -> tuple:
    return (cfg.mode, getattr(cfg, "channel", ""), getattr(cfg, "arm_type", "yam_pro"),
            getattr(cfg, "gripper_type", "linear_4310"))


def acquire(cfg, *, free=None) -> "YamSession":
    """Get or create the shared session.

    free=True asks for a hand-movable chain (gravity compensation, no position targets), which
    is what the read-only feedback reader needs while teaching. A session that already drives the
    arm ignores the request, so preview can never fight control.
    """
    key = session_key(cfg)
    with _SESSIONS_LOCK:
        session = _SESSIONS.get(key)
        if session is None:
            session = YamSession(cfg)
            _SESSIONS[key] = session
        session.refs += 1
    if free is not None and not session.arm_attached:
        session.set_free(free)
    session.start()
    return session


def release(session: "YamSession") -> None:
    """Drop one view of the session. Never let the arm go limp.

    i2rt's close() zeroes every torque, which drops a raised YAM instantly. So the last release
    parks the session in a servo hold of the measured pose and keeps the chain (and the gravity
    compensation) alive. Only process shutdown calls stop().
    """
    with _SESSIONS_LOCK:
        session.refs = max(0, session.refs - 1)
        remaining = session.refs
    if remaining == 0:
        session.park()


def shutdown_all() -> None:
    """Close every chain exactly once, at interpreter exit."""
    with _SESSIONS_LOCK:
        sessions = list(_SESSIONS.values())
        _SESSIONS.clear()
    for session in sessions:
        try:
            session.stop()
        except Exception:  # noqa: BLE001 - shutdown must not raise
            pass


atexit.register(shutdown_all)


class YamSession:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.key = session_key(cfg)
        self.refs = 0
        self.backend = None
        self.kin = None
        self.control_hz = float(getattr(cfg, "control_hz", 100.0))
        self.max_joint_speed = float(getattr(cfg, "max_joint_speed", 2.0))
        self.last_error = None
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._thread = None
        self._q_cmd = None
        self._pose_target = None
        self._gripper_target = 0.0
        self.free = False
        self.arm_attached = False
        self._debug_path = os.environ.get("ROBOAPI_YAM_LOG", "")
        self._debug_last = 0.0
        self._mode_file = os.environ.get(
            "ROBOAPI_YAM_MODE_FILE", os.path.expanduser("~/roboapi/state/yam_mode"))
        self._mode_mtime = None
        self._started_at = 0.0

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if self.backend is not None:
            # The control loop died (i2rt thread error): rebuild the chain instead of reusing it.
            try:
                self.backend.close()
            except Exception:  # noqa: BLE001
                pass
            self.backend = None
        if self.backend is None:
            self.backend = create_backend(
                self.cfg.mode,
                channel=getattr(self.cfg, "channel", ""),
                arm_type=getattr(self.cfg, "arm_type", "yam_pro"),
                gripper_type=getattr(self.cfg, "gripper_type", "linear_4310"),
                gripper_limits=getattr(self.cfg, "gripper_limits", None),
            )
            self.kin = kinematics_for(getattr(self.cfg, "arm_type", "yam_pro"))
        q7 = self.backend.joints7()
        self._q_cmd = np.asarray(q7[:6], dtype=float).copy()
        if self.free:
            self.backend.free_mode()
        self._gripper_target = float(q7[6]) if q7.shape[0] > 6 else 0.0
        self._pose_target = self.kin.fk(self._q_cmd)
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="yam-control", daemon=True)
        self._thread.start()
        self._started_at = time.monotonic()

    def park(self) -> None:
        """Leave the arm hand-movable (gravity compensation) when nobody is driving it.

        Holding a pose is safer for a task, but for an operator who needs to park the arm by hand
        gravity compensation is the useful default; the mode file (below) can ask for hold instead.
        """
        if self.backend is None:
            return
        try:
            self.set_free(True)
        except Exception as exc:  # noqa: BLE001
            self.last_error = "park failed: " + type(exc).__name__ + ": " + str(exc)

    def sync_mode_file(self, value: str) -> None:
        """Record the mode the runtime just put us in, so a stale switch cannot override it."""
        try:
            os.makedirs(os.path.dirname(self._mode_file), exist_ok=True)
            with open(self._mode_file, "w", encoding="utf-8") as fh:
                fh.write(value + chr(10))
            self._mode_mtime = os.stat(self._mode_file).st_mtime
        except OSError:
            pass

    def _apply_mode_file(self) -> None:
        """Operator switch: gravity (hand-movable), hold (servo-hold here), servo (runtime targets)."""
        try:
            stat = os.stat(self._mode_file)
        except OSError:
            return
        if stat.st_mtime == self._mode_mtime:
            return
        self._mode_mtime = stat.st_mtime
        try:
            with open(self._mode_file, encoding="utf-8") as fh:
                value = fh.read().strip().lower()
        except OSError:
            return
        if value == "gravity":
            self.set_free(True)
            _trace("mode -> gravity")
        elif value == "hold":
            self.set_free(False)
            self.set_pose(self.pose7())
            _trace("mode -> hold")
        elif value == "servo":
            if self.free:
                self.set_free(False)
            _trace("mode -> servo")

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.backend is not None:
            self.backend.close()
            self.backend = None

    # -- views -------------------------------------------------------------
    @property
    def live(self) -> bool:
        return self._running.is_set() and self.last_error is None

    def q7(self) -> np.ndarray:
        return self.backend.joints7()

    def q6(self) -> np.ndarray:
        return np.asarray(self.backend.joints7()[:6], dtype=float)

    def pose7(self) -> np.ndarray:
        return self.kin.fk(self.q6())

    def ratio(self) -> float:
        q7 = self.backend.joints7()
        return float(np.clip(q7[6], 0.0, 1.0)) if q7.shape[0] > 6 else 0.0

    def set_free(self, free: bool) -> None:
        """Hand-movable (no targets) versus servo control of the same session."""
        _trace("session.set_free %d -> %d" % (int(self.free), int(bool(free))))
        self.free = bool(free)
        if self.free:
            self._pose_target = None
            if self.backend is not None:
                self.backend.free_mode()

    def set_pose(self, pose7) -> None:
        pose = np.asarray(pose7, dtype=float).reshape(-1)
        if pose.shape != (7,):
            raise ValueError(f"servo_pose expects 7 values, got {pose.shape}")
        with self._lock:
            self._pose_target = pose

    def set_ratio(self, ratio: float) -> None:
        with self._lock:
            self._gripper_target = float(np.clip(ratio, 0.0, 1.0))

    def validate_device(self, device) -> None:
        expected = self.backend.identity() if self.backend is not None else {}
        for key, value in expected.items():
            if key in (device or {}) and device[key] != value:
                raise ValueError(f"calibration was recorded for {key}={device[key]!r}, "
                                 f"this robot reports {value!r}")

    # -- control loop ------------------------------------------------------
    def _loop(self) -> None:
        period = 1.0 / max(self.control_hz, 1.0)
        max_step = self.max_joint_speed * period
        deadline = time.perf_counter()
        while self._running.is_set():
            try:
                self._apply_mode_file()
                q7 = self.backend.joints7()
                with self._lock:
                    target = None if self._pose_target is None else np.array(self._pose_target)
                    gripper = self._gripper_target
                if self.free:
                    # Hand-movable: never send position targets, just track the measured joints.
                    self._q_cmd = np.asarray(q7[:6], dtype=float).copy()
                elif target is not None:
                    # Joint PD droops under gravity (~5-10 mm at the flange). Aim past the target
                    # by the measured error so the servo settles on it instead of below it.
                    measured_now = self.kin.fk(np.asarray(q7[:6], dtype=float))
                    aim = np.array(target, dtype=float)
                    droop = np.clip(measured_now[:3] - target[:3], -0.02, 0.02)
                    aim[:3] = target[:3] - 0.6 * droop
                    ok, q_sol = self.kin.solve_ik(aim, self._q_cmd, dt=0.05, max_iters=14,
                                                  pos_tol=2e-4, ori_tol=2e-3)
                    step = np.clip(q_sol - self._q_cmd, -max_step, max_step)
                    self._q_cmd = np.clip(self._q_cmd + step, self.kin.low, self.kin.high)
                else:
                    self._q_cmd = np.asarray(q7[:6], dtype=float)
                if not self.free:
                    cmd = np.concatenate([self._q_cmd, [gripper]])
                    self.backend.command7(cmd)
                if self._debug_path:
                    now = time.monotonic()
                    if now - self._debug_last >= 0.5:
                        self._debug_last = now
                        measured = self.kin.fk(np.asarray(q7[:6], dtype=float))
                        tgt = "none" if target is None else np.round(target[:3], 4).tolist()
                        got = np.round(measured[:3], 4).tolist()
                        qc = np.round(np.asarray(self._q_cmd)[:3], 3).tolist()
                        qm = np.round(np.asarray(q7[:3], dtype=float), 3).tolist()
                        msg = "t=" + format(now, ".1f") + " free=" + str(int(self.free))
                        msg += " target=" + str(tgt) + " measured=" + str(got)
                        msg += " qcmd=" + str(qc) + " qmeas=" + str(qm)
                        err = 0.0 if target is None else float(np.linalg.norm(measured[:3] - target[:3]))
                        msg += " err=" + format(err, ".4f")
                        with open(self._debug_path, "a", encoding="utf-8") as fh:
                            fh.write(msg + chr(10))
                if self.last_error is not None:
                    self.last_error = None
            except Exception as exc:  # noqa: BLE001 - surfaced through last_error
                self.last_error = f"{type(exc).__name__}: {exc}"
                _trace("loop error " + self.last_error)
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
    try:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write("EVENT " + event + chr(10))
    except Exception:
        pass
