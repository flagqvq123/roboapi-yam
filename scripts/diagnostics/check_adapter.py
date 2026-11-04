#!/usr/bin/env python3
"""Dry-run check of the YAM adapter stack without RoboAPI: arm, gripper, camera, timing."""
import time

import numpy as np

from roboapi_yam.arm import YamArm, YamArmConfig
from roboapi_yam.camera import YamCamera, YamCameraConfig
from roboapi_yam.gripper import YamGripper, YamGripperConfig


def main():
    ac = YamArmConfig(mode="sim")
    ac.validate()
    arm = YamArm(ac)
    t0 = time.time()
    arm.connect()
    print("connect_s", round(time.time() - t0, 3))
    print("live", arm.live, "joints", np.round(arm.get_joints(), 4))
    p0 = arm.get_eef_pose()
    print("pose0", np.round(p0, 5))

    target = p0.copy()
    target[1] += 0.05
    delays = []
    for _ in range(60):
        t = time.perf_counter()
        arm.servo_pose(target)
        delays.append((time.perf_counter() - t) * 1e6)
        time.sleep(0.02)
    time.sleep(0.8)
    p1 = arm.get_eef_pose()
    print("pose1", np.round(p1, 5))
    print("delta_xyz_m", np.round(p1[:3] - p0[:3], 5), "servo_pose_us_mean", round(float(np.mean(delays)), 1))

    gc = YamGripperConfig(mode="sim")
    gc.validate()
    grip = YamGripper(gc)
    grip.connect()
    print("grip_live", grip.live, "ratio0", round(grip.get_ratio(), 3))
    grip.set_ratio(0.05)
    time.sleep(0.7)
    print("ratio1", round(grip.get_ratio(), 3))

    cc = YamCameraConfig(source="synthetic")
    cc.validate()
    cam = YamCamera(cc)
    cam.connect()
    time.sleep(0.3)
    img, age = cam.read()
    print("frame", img.shape, img.dtype, "age_ms", round(age * 1000, 1))
    first = cam.read()[1]
    t = time.perf_counter()
    nxt = cam.read_after(first, 1.0)
    print("read_after_ok", nxt is not None, "wait_ms", round((time.perf_counter() - t) * 1000, 1))

    arm.disconnect()
    grip.disconnect()
    cam.disconnect()
    print("disconnected ok")


if __name__ == "__main__":
    main()
