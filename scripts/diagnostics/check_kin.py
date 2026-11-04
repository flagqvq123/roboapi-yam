#!/usr/bin/env python3
"""Check YAM flange kinematics: FK sanity, IK round-trip, IK timing."""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "roboapi-yam", "src"))

from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

from roboapi_yam.kin import YamKinematics, matrix_to_pose7, pose7_to_matrix

ARM = ArmType.YAM_PRO


def main():
    path = combine_arm_and_gripper_xml(ARM, GripperType.NO_GRIPPER)
    kin = YamKinematics(path)
    print("joint_count", kin.joint_count)
    print("low ", np.round(kin.low, 4))
    print("high", np.round(kin.high, 4))
    p0 = kin.fk(np.zeros(6))
    print("fk(q=0) pose7 wxyz:", np.round(p0, 6), "quat_norm", round(float(np.linalg.norm(p0[3:])), 6))

    rng = np.random.default_rng(0)
    lo = np.maximum(kin.low, -1.2)
    hi = np.minimum(kin.high, 1.2)
    ok_n = 0
    times, pos_err, ori_err = [], [], []
    q_prev = np.zeros(6)
    for i in range(30):
        q_target = rng.uniform(lo * 0.7, hi * 0.7)
        target = kin.fk(q_target)
        t0 = time.perf_counter()
        ok, q_sol = kin.solve_ik(target, q_prev)
        dt = (time.perf_counter() - t0) * 1000
        back = kin.fk(q_sol)
        Tt, Tb = pose7_to_matrix(target), pose7_to_matrix(back)
        e_pos = float(np.linalg.norm(Tt[:3, 3] - Tb[:3, 3]))
        e_ori = float(np.linalg.norm(Tt[:3, :3] @ Tb[:3, :3].T - np.eye(3)))
        times.append(dt); pos_err.append(e_pos); ori_err.append(e_ori)
        ok_n += int(ok)
        q_prev = q_sol
        if i < 5:
            print("  trial%d ok=%s ik_ms=%6.2f pos_err=%6.3fmm ori=%.2e" % (i, ok, dt, e_pos*1000, e_ori))
    print("IK: %d/30 ok, mean %.2f ms p95 %.2f ms max %.2f ms" % (ok_n, np.mean(times), np.percentile(times,95), np.max(times)))
    print("    pos_err mean %.3f mm max %.3f mm" % (np.mean(pos_err)*1000, np.max(pos_err)*1000))
    print("    ori_err mean %.2e max %.2e" % (np.mean(ori_err), np.max(ori_err)))

    q = np.zeros(6)
    p = kin.fk(q)
    times = []
    for i in range(50):
        T = pose7_to_matrix(p)
        T[1, 3] += 0.002
        t0 = time.perf_counter()
        ok, q = kin.solve_ik(matrix_to_pose7(T), q, max_iters=12)
        times.append((time.perf_counter() - t0) * 1000)
        p = kin.fk(q)
    print("small-step IK (12 iters): mean %.2f ms max %.2f ms" % (np.mean(times), np.max(times)))
    print("final pose:", np.round(p, 5))


if __name__ == "__main__":
    main()
