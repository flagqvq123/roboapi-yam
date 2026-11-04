"""YAM parallel gripper: 0 = closed, 1 = open, shared CAN session with the arm."""

from __future__ import annotations

import time
from dataclasses import dataclass

from roboapi.hardware import ComponentOption, ComponentPlugin, GripperConfig

from .session import acquire, release

ARM_TYPES = ("yam", "yam_pro", "yam_ultra", "yam_ultra_2", "big_yam")
GRIPPER_TYPES = ("linear_4310", "linear_3507", "crank_4310", "flexible_4310", "no_gripper")


@dataclass
class YamGripperConfig(GripperConfig):
    type: str = "yam"
    mode: str = "sim"
    channel: str = "can_follower_r"
    arm_type: str = "yam_pro"
    gripper_type: str = "linear_4310"
    # [closed, open] motor radians. Linear grippers need calibrated travels; when the site already
    # knows them (a production stack that runs the same arm), pass them to skip i2rt calibration.
    gripper_limits: list[float] | None = None

    @property
    def connection_id(self) -> str:
        return (f"sim:{self.arm_type}" if self.mode == "sim"
                else f"can:{self.channel}:{self.arm_type}")

    def validate(self) -> None:
        super().validate()
        if self.mode not in ("sim", "real"):
            raise ValueError("gripper.mode must be 'sim' or 'real'")
        if self.arm_type not in ARM_TYPES:
            raise ValueError(f"gripper.arm_type must be one of {ARM_TYPES}")
        if self.gripper_type not in GRIPPER_TYPES:
            raise ValueError(f"gripper.gripper_type must be one of {GRIPPER_TYPES}")
        if self.gripper_limits is not None:
            limits = self.gripper_limits
            if (not isinstance(limits, (list, tuple)) or len(limits) != 2
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in limits)
                    or limits[0] == limits[1]):
                raise ValueError("gripper.gripper_limits must be two travel endpoints in radians")


def configure(option, setup):
    return option.config_type(mode="sim",
                              channel=setup.ask("CAN channel (real mode only)", "can_follower_r"))


def gripper_plugin():
    return ComponentPlugin(
        options=(ComponentOption("YAM gripper (simulation)", YamGripperConfig, {"mode": "sim"}),
                 ComponentOption("YAM gripper (SocketCAN)", YamGripperConfig, {"mode": "real"})),
        create=YamGripper, configure=configure,
        setup_notes=("Gripper shares the arm's CAN session; keep mode/channel/arm_type identical.",),
    )


class YamGripper:
    def __init__(self, cfg):
        self.cfg = cfg
        self._session = None
        self.last_error = None

    @property
    def live(self) -> bool:
        return self._session is not None and self._session.live

    def connect(self) -> None:
        _trace("gripper.connect")
        if self._session is not None:
            return
        self._session = acquire(self.cfg)

    def disconnect(self) -> None:
        _trace("gripper.disconnect")
        if self._session is not None:
            release(self._session)
            self._session = None

    def get_ratio(self) -> float:
        return self._session.ratio()

    def set_ratio(self, ratio: float) -> None:
        self._session.set_ratio(ratio)

    def move_blocking(self, ratio: float, should_stop=None) -> None:
        self._session.set_ratio(ratio)
        timeout_s = max(1.0, float(self.cfg.full_travel_s) * 3.0)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            stop = should_stop() if callable(should_stop) else bool(should_stop)
            if stop:
                return
            if abs(self._session.ratio() - float(ratio)) < 0.03:
                return
            time.sleep(0.01)

    def hold(self) -> None:
        if self._session is not None:
            self._session.set_ratio(self._session.ratio())

    def recover(self) -> None:
        pass

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
