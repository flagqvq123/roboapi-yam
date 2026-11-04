#!/usr/bin/env python3
"""Write a synthetic calibration profile for the sim YAM (no hardware, no teaching).

The runtime only checks that the profile's Home/plane geometry is consistent and that its
identity hash matches this installation, so we derive the four native poses from FK and store
them the same way the console would.
"""
import hashlib
import json

import numpy as np

from roboapi.calibration.profile import CalibrationProfile, save_profile
from roboapi.config.loader import load

from roboapi_yam.session import kinematics_for


def main():
    cfg = load("config-sim.yaml")
    identity = hashlib.sha256(json.dumps([
        cfg.arm.type, cfg.arm.connection_id, cfg.calibration.tool_id,
        cfg.gripper.type, cfg.gripper.connection_id,
        cfg.camera.type, cfg.camera.connection_id,
    ]).encode()).hexdigest()

    kin = kinematics_for(cfg.arm.arm_type)

    def pose(q):
        return [float(v) for v in kin.fk(np.asarray(q, dtype=float))]

    q_home = np.zeros(6)
    q_a = np.array([0.10, 0.10, 0.05, 0.00, 0.10, 0.00])
    q_b = q_a + np.array([0.20, 0.00, 0.00, 0.00, 0.00, 0.00])
    q_c = q_a + np.array([0.00, 0.15, 0.05, 0.00, 0.05, 0.00])

    home = pose(q_home)
    plane = [pose(q_a), pose(q_b), pose(q_c)]
    device = {"backend": "sim", "arm_type": cfg.arm.arm_type, "gripper_type": cfg.arm.gripper_type}

    profile = CalibrationProfile(identity=identity, device=device, home_native_pose7=home,
                                 plane_native_poses7=plane, check_native_pose7=home)
    adapter = profile.adapter()
    save_profile(profile, cfg.calibration.profile_file)
    print("identity", identity[:16], "...")
    print("home", np.round(home, 5))
    print("plane", [np.round(p[:3], 5).tolist() for p in plane])
    print("task_from_base_R", np.round(adapter.task_from_base[:3, :3], 4).tolist())
    print("saved to", cfg.calibration.profile_file)


if __name__ == "__main__":
    main()
