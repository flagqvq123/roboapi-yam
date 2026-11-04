#!/usr/bin/env python3
"""Check the free/feedback path and that servo still works afterwards (sim)."""
import time

import numpy as np

from roboapi_yam.arm import YamArm, YamArmConfig, YamFeedback


def main():
    cfg = YamArmConfig(mode="sim")
    cfg.validate()

    feedback = YamFeedback(cfg)
    feedback.connect()
    first = feedback.snapshot()
    time.sleep(0.3)
    second = feedback.snapshot()
    print("feedback live", feedback.live)
    print("pose", np.round(first["pose"], 5))
    print("device", first["device"])
    print("timestamps increase", second["timestamp"] > first["timestamp"])
    session = feedback._session
    print("session free at feedback", session.free)

    arm = YamArm(cfg)
    arm.connect()
    print("session free after arm connect", session.free, "arm_attached", session.arm_attached)
    p0 = arm.get_eef_pose()
    target = p0.copy()
    target[0] += 0.03
    for _ in range(40):
        arm.servo_pose(target)
        time.sleep(0.02)
    time.sleep(0.8)
    p1 = arm.get_eef_pose()
    print("servo delta_xyz", np.round(p1[:3] - p0[:3], 4), "norm", round(float(np.linalg.norm(p1[:3] - p0[:3])), 4))

    moved = feedback.snapshot()
    print("feedback sees the move", np.allclose(moved["pose"][:3], p1[:3], atol=1e-6))

    arm.disconnect()
    feedback.disconnect()
    print("session closed", session._thread is None)


if __name__ == "__main__":
    main()
