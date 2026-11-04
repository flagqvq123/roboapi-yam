"""Flange-centre kinematics for YAM arms (MuJoCo + mink). No hardware access."""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "disable")

import numpy as np

import mujoco
from scipy.spatial.transform import Rotation

FRAME_NAME = "gripper"  # i2rt: child of joint6 == end-effector mount == flange centre
FRAME_TYPE = "body"


def pose7_to_matrix(pose7):
    """[x, y, z, qw, qx, qy, qz] -> 4x4 homogeneous transform."""
    p = np.asarray(pose7, dtype=float).reshape(-1)
    if p.shape != (7,):
        raise ValueError(f"expected 7 values, got {p.shape}")
    w, x, y, z = p[3], p[4], p[5], p[6]
    n = np.linalg.norm([w, x, y, z])
    if not np.isfinite(n) or n < 1e-9:
        raise ValueError("quaternion is not finite/non-zero")
    w, x, y, z = w / n, x / n, y / n, z / n
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    T[:3, 3] = p[:3]
    return T


def matrix_to_pose7(T):
    T = np.asarray(T, dtype=float)
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    return np.array([T[0, 3], T[1, 3], T[2, 3], qw, qx, qy, qz])


class YamKinematics:
    """6-DoF arm model; the flange frame is the MJCF body named 'gripper'.

    Gripper fingers are omitted on purpose: they do not move the flange, and a
    6-DoF model keeps the IK QP small and the joint vector equal to the motor chain.
    """

    def __init__(self, model_xml, *, joint_count=6):
        self.model = mujoco.MjModel.from_xml_path(str(model_xml))
        self.data = mujoco.MjData(self.model)
        if self.model.nq != joint_count:
            raise ValueError(f"expected {joint_count} joints, model has {self.model.nq}")
        self.joint_count = joint_count
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, FRAME_NAME)
        if self.body_id < 0:
            raise ValueError(f"model has no body named {FRAME_NAME!r}")
        self.low = self.model.jnt_range[:, 0].copy()
        self.high = self.model.jnt_range[:, 1].copy()

        import mink

        self._mink = mink
        self.configuration = mink.Configuration(self.model)
        limits = []
        try:
            limits.append(mink.ConfigurationLimit(self.model))
        except Exception:
            pass
        self.limits = limits

    def fk(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape != (self.joint_count,):
            raise ValueError(f"expected {self.joint_count} joints, got {q.shape}")
        self.data.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.data)
        xyz = self.data.xpos[self.body_id]
        qw, qx, qy, qz = self.data.xquat[self.body_id]
        return np.array([xyz[0], xyz[1], xyz[2], qw, qx, qy, qz])

    def solve_ik(self, pose7, q_init=None, *, dt=0.02, max_iters=40,
                 pos_tol=1e-4, ori_tol=1e-3, damping=1e-6):
        mink = self._mink
        T = pose7_to_matrix(pose7)
        if q_init is not None:
            self.configuration.update(np.asarray(q_init, dtype=float))
        task = mink.FrameTask(FRAME_NAME, FRAME_TYPE, position_cost=1.0, orientation_cost=1.0,
                              lm_damping=1.0)
        task.set_target(mink.SE3.from_matrix(T))
        err = None
        for _ in range(max_iters):
            vel = mink.solve_ik(self.configuration, [task], dt, "quadprog", damping=damping,
                                limits=self.limits)
            self.configuration.integrate_inplace(vel, dt)
            err = task.compute_error(self.configuration)
            if np.linalg.norm(err[:3]) <= pos_tol and np.linalg.norm(err[3:]) <= ori_tol:
                break
        q = np.clip(np.asarray(self.configuration.q, dtype=float), self.low, self.high)
        ok = bool(np.linalg.norm(err[:3]) <= max(pos_tol, 5e-3)
                  and np.linalg.norm(err[3:]) <= max(ori_tol, 5e-2))
        return ok, q
