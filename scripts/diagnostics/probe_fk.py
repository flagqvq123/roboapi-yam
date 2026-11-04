#!/usr/bin/env python3
"""Read-only probes of the YAM kinematic model (no hardware, no CAN).

Answers: where is the flange frame, what joint order/qpos layout the MuJoCo model
has, and how far the gripper TCP sits from the flange.
"""
import os
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "disable")

import mujoco


def load(arm_name: str, gripper_name: str):
    from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

    arm = ArmType.from_string_name(arm_name)
    grip = GripperType(gripper_name)
    path = combine_arm_and_gripper_xml(arm, grip)
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    return arm, grip, path, model, data


def describe(model, data, label):
    print(f"\n== {label} ==")
    print("nq", model.nq, "nv", model.nv, "nbody", model.nbody, "nsite", model.nsite)
    print("joints (qposadr order):")
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        print(f"  {j} {name:10s} qposadr={model.jnt_qposadr[j]} dofadr={model.jnt_dofadr[j]} "
              f"type={model.jnt_type[j]} range={model.jnt_range[j]}")
    print("bodies:", [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(model.nbody)])
    print("sites :", [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, s) for s in range(model.nsite)])


def pose_of_body(model, data, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return np.concatenate([data.xpos[bid], data.xquat[bid]])


def pose_of_site(model, data, name):
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    return np.concatenate([data.xpos[sid], data.xquat[sid]])


def main():
    for arm_name, grip_name in (("yam_pro", "linear_4310"), ("yam", "linear_4310")):
        try:
            arm, grip, path, model, data = load(arm_name, grip_name)
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] {arm_name}/{grip_name}: {type(exc).__name__}: {exc}")
            continue
        print(f"\n{'='*70}\narm={arm.value} gripper={grip.value}\nxml={path}")
        describe(model, data, "structure")

        n_arm = 6
        mujoco.mj_forward(model, data)
        flange0 = pose_of_body(model, data, "gripper")
        print("\nq=0 flange (body 'gripper') xyz+wxyz:", np.round(flange0, 6))
        for s in [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i) for i in range(model.nsite)]:
            p = pose_of_site(model, data, s)
            print(f"  site {s:12s} xyz={np.round(p[:3],6)} d_xyz={np.round(p[:3]-flange0[:3],6)} |d|={np.linalg.norm(p[:3]-flange0[:3]):.4f}")

        # a bent pose: only arm joints, keep gripper joint wherever it is
        q = np.zeros(model.nq)
        q[:min(6, model.nq)] = [0.3, 0.4, -0.5, 0.8, 0.2, 0.1][:min(6, model.nq)]
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        fl = pose_of_body(model, data, "gripper")
        print("\nq=[0.3,0.4,-0.5,0.8,0.2,0.1] flange:", np.round(fl, 6))
        print("quat norm:", np.linalg.norm(fl[3:]))

        # Jacobian / IK feasibility probe: numeric dq that moves flange +1cm in x
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        mujoco.mj_jacBody(model, data, jacp, jacr, bid)
        print("flange linear vel per unit qdot (first 6 cols):\n", np.round(jacp[:, :6], 4))


if __name__ == "__main__":
    main()
