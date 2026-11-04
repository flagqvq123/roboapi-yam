"""Device I/O for YAM arms: one CAN session (or a MuJoCo stand-in) behind a tiny protocol.

Nothing here touches hardware at import time: i2rt is imported inside the constructor so the
adapter can be discovered by RoboAPI without a robot, a CAN interface or a vendor SDK install.
"""

from __future__ import annotations

import numpy as np

MODES = ("sim", "real")


def _enum(name, value):
    from i2rt.robots import utils

    enum_cls = getattr(utils, name)
    try:
        return enum_cls(value)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"unknown {name} {value!r}: {exc}") from None


class YamBackend:
    """Minimal joint-level view of a YAM chain (6 arm joints + 1 normalised gripper)."""

    mode = "unknown"

    def joints7(self) -> np.ndarray:
        raise NotImplementedError

    def command7(self, cmd7) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def identity(self) -> dict:
        return {"backend": self.mode}

    def gravity_comp(self) -> None:
        pass

    def free_mode(self) -> None:
        """Leave the chain hand-movable (no position targets): gravity compensation by default."""
        self.gravity_comp()

    def zero_torque(self) -> None:
        pass


class SimBackend(YamBackend):
    """MuJoCo stand-in from i2rt, same Robot protocol as the real chain."""

    mode = "sim"

    def __init__(self, arm_type: str, gripper_type: str) -> None:
        from i2rt.robots.get_robot import get_yam_robot

        self.arm_type = arm_type
        self.gripper_type = gripper_type
        self._robot = get_yam_robot(sim=True, arm_type=_enum("ArmType", arm_type),
                                    gripper_type=_enum("GripperType", gripper_type))

    def joints7(self) -> np.ndarray:
        return np.asarray(self._robot.get_joint_pos(), dtype=float)

    def command7(self, cmd7) -> None:
        self._robot.command_joint_pos(np.asarray(cmd7, dtype=float))

    def close(self) -> None:
        try:
            self._robot.close()
        except Exception:  # noqa: BLE001
            pass

    def identity(self) -> dict:
        return {"backend": "sim", "arm_type": self.arm_type, "gripper_type": self.gripper_type}

    def gravity_comp(self) -> None:
        try:
            self._robot.enter_gravity_comp_idle()
        except Exception:  # noqa: BLE001
            pass

    def zero_torque(self) -> None:
        try:
            self._robot.enter_zero_torque_idle()
        except Exception:  # noqa: BLE001
            pass


class CanBackend(YamBackend):
    """A real YAM on a SocketCAN interface (1 Mbit/s, DM motors)."""

    mode = "real"

    def __init__(self, channel: str, arm_type: str, gripper_type: str,
                 gripper_limits=None) -> None:
        from i2rt.robots.get_robot import get_yam_robot

        kwargs = {}
        if gripper_limits is not None:
            kwargs["gripper_limits_override"] = np.asarray(gripper_limits, dtype=float)
        self.channel = channel
        self.arm_type = arm_type
        self.gripper_type = gripper_type
        self._robot = get_yam_robot(channel=channel, arm_type=_enum("ArmType", arm_type),
                                    gripper_type=_enum("GripperType", gripper_type), **kwargs)

    def joints7(self) -> np.ndarray:
        return np.asarray(self._robot.get_joint_pos(), dtype=float)

    def command7(self, cmd7) -> None:
        self._robot.command_joint_pos(np.asarray(cmd7, dtype=float))

    def close(self) -> None:
        try:
            self._robot.close()
        except Exception:  # noqa: BLE001
            pass

    def identity(self) -> dict:
        return {"backend": "can", "channel": self.channel, "arm_type": self.arm_type,
                "gripper_type": self.gripper_type}

    def gravity_comp(self) -> None:
        try:
            self._robot.enter_gravity_comp_idle()
        except Exception:  # noqa: BLE001
            pass

    def zero_torque(self) -> None:
        try:
            self._robot.enter_zero_torque_idle()
        except Exception:  # noqa: BLE001
            pass


def create_backend(mode: str, *, channel: str = "", arm_type: str = "yam_pro",
                   gripper_type: str = "linear_4310", gripper_limits=None) -> YamBackend:
    if mode == "sim":
        return SimBackend(arm_type, gripper_type)
    if mode == "real":
        if not channel:
            raise ValueError("real mode needs a CAN channel, e.g. can_follower_r")
        return CanBackend(channel, arm_type, gripper_type, gripper_limits)
    raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
