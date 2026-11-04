"""YAM arm: RoboAPI Cartesian interface on top of the i2rt joint-position chain."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from roboapi.hardware import ArmConfig, ArmPlugin, ComponentOption

from .session import acquire, release

ARM_TYPES = ("yam", "yam_pro", "yam_ultra", "yam_ultra_2", "big_yam")
GRIPPER_TYPES = ("linear_4310", "linear_3507", "crank_4310", "flexible_4310", "no_gripper")


def _connection_id(mode: str, channel: str, arm_type: str) -> str:
    return f"sim:{arm_type}" if mode == "sim" else f"can:{channel}:{arm_type}"


def _expected_identity(cfg) -> dict:
    """Device identity as the feedback reader reports it; usable without a connection."""
    if cfg.mode == "sim":
        return {"backend": "sim", "arm_type": cfg.arm_type, "gripper_type": cfg.gripper_type}
    return {"backend": "can", "channel": cfg.channel, "arm_type": cfg.arm_type,
            "gripper_type": cfg.gripper_type}


def _check_device(cfg, device) -> None:
    for key, value in _expected_identity(cfg).items():
        if key in (device or {}) and device[key] != value:
            raise ValueError(f"calibration was recorded for {key}={device[key]!r}, "
                             f"this robot reports {value!r}")


@dataclass
class YamArmConfig(ArmConfig):
    type: str = "yam"
    mode: str = "sim"                  # "sim" (MuJoCo) or "real" (SocketCAN)
    channel: str = "can_follower_r"    # ignored in sim
    arm_type: str = "yam_pro"
    gripper_type: str = "linear_4310"
    control_hz: float = 100.0
    max_joint_speed: float = 2.0       # rad/s, slew limit on the joint target
    gripper_limits: list[float] | None = field(default=None)  # [closed, open] rad, real mode only

    @property
    def connection_id(self) -> str:
        return _connection_id(self.mode, self.channel, self.arm_type)

    def validate(self) -> None:
        super().validate()
        if self.mode not in ("sim", "real"):
            raise ValueError("arm.mode must be 'sim' or 'real'")
        if self.arm_type not in ARM_TYPES:
            raise ValueError(f"arm.arm_type must be one of {ARM_TYPES}")
        if self.gripper_type not in GRIPPER_TYPES:
            raise ValueError(f"arm.gripper_type must be one of {GRIPPER_TYPES}")
        for name in ("control_hz", "max_joint_speed"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0:
                raise ValueError(f"arm.{name} must be a positive number")


def configure(option, setup):
    mode = ("sim", "real")[setup.choose(
        "Robot backend", ["MuJoCo simulation (no hardware)", "SocketCAN (real YAM)"])]
    channel = setup.ask("CAN channel (real mode only)", "can_follower_r")
    arm_type = ARM_TYPES[setup.choose("YAM model", list(ARM_TYPES))]
    return option.config_type(mode=mode, channel=channel, arm_type=arm_type)


def arm_plugin():
    return ArmPlugin(
        options=(ComponentOption("YAM (MuJoCo simulation)", YamArmConfig, {"mode": "sim"}),
                 ComponentOption("YAM (SocketCAN)", YamArmConfig, {"mode": "real"})),
        create=YamArm, create_feedback=YamFeedback, configure=configure,
        setup_notes=("Simulation mode needs no hardware; switch arm.mode to real when the arm is free.",
                     ),
    )


class YamArm:
    """RoboAPI's Arm protocol; every call is forwarded to the shared session."""

    def __init__(self, cfg, tool=None):
        self.cfg = cfg
        self.tool = tool
        self._session = None
        self.last_error = None

    @property
    def live(self) -> bool:
        return self._session is not None and self._session.live

    def connect(self, *, calibration: bool = False) -> None:
        _trace("arm.connect enter")
        if self._session is not None:
            self._session.set_free(False)
            self._session.set_pose(self._session.pose7())
            self._session.sync_mode_file("servo")
            _trace("arm.connect already connected")
            return
        session = acquire(self.cfg)
        session.arm_attached = True
        session.set_free(False)
        session.set_pose(session.pose7())
        session.sync_mode_file("servo")
        self._session = session
        _trace("arm.connect done free=0")

    def disconnect(self) -> None:
        _trace("arm.disconnect")
        if self._session is not None:
            self._session.arm_attached = False
            release(self._session)
            self._session = None

    def get_joints(self) -> np.ndarray:
        return self._session.q6()

    def get_eef_pose(self) -> np.ndarray:
        """Measured native-base flange pose, [x, y, z, qw, qx, qy, qz]."""
        return self._session.pose7()

    def servo_pose(self, pose7, twist=None, *, position_only: bool = False) -> None:
        self._session.set_pose(pose7)

    def validate_calibration(self, device: dict) -> None:
        _check_device(self.cfg, device)

    def recover(self, should_stop=None) -> None:
        self.disconnect()
        self.connect()
        if callable(should_stop) and should_stop():
            self.disconnect()

    def hold(self) -> None:
        if self._session is not None:
            self._session.set_pose(self._session.pose7())

    def check_motion_ready(self) -> None:
        """Prerequisites checked before an explicit robot operation (not a connection test)."""
        if self.cfg.mode == "real":
            import os

            if not os.path.isdir(f"/sys/class/net/{self.cfg.channel}"):
                raise RuntimeError(f"CAN interface {self.cfg.channel} is not available")


class YamFeedback:
    """Read-only flange feedback.

    In simulation the device is shared with the control session (no hardware is claimed); on a
    real CAN chain this still has to become a passive-encoder reader, which is why real mode
    refuses instead of silently taking motion control.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._session = None
        self.last_error = None

    @property
    def live(self) -> bool:
        return self._session is not None and self._session.live

    def connect(self) -> None:
        _trace("feedback.connect enter")
        # Free mode: the chain is gravity-compensated and hand-movable and no position targets are
        # streamed, so reading the state does not take control away from the operator.
        self._session = acquire(self.cfg, free=True)

    def disconnect(self) -> None:
        _trace("feedback.disconnect")
        if self._session is not None:
            release(self._session)
            self._session = None

    def snapshot(self) -> dict:
        pose = self._session.pose7()
        return {
            "pose": [float(v) for v in pose],
            "timestamp": time.monotonic(),
            "device": self._session.backend.identity(),
            "has_errors": False,
        }

    def validate_device(self, device: dict) -> None:
        _check_device(self.cfg, device)

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
