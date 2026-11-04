# YAM 适配事实记录（2026-11-03）

## 运动学（MuJoCo 模型 i2rt/robot_models/arm/yam_pro）

- 法兰中心 = MJCF body `gripper`（joint6 的子节点），等价于 site `tcp_site`（pos 0 0 0）。
- 工具中心 TCP = site `grasp_site`，位于法兰系 +Z 方向 **0.14465 m**。
- 组装模型（arm+gripper）nq=8：joint1..joint6 + joint7/joint8（两个滑爪，等约束，行程 0–0.0475 m）。
  法兰位姿与手指无关，因此标定/伺服用 `no_gripper` 的 6 自由度模型即可。
- 关节限位（rad）：j1 [-2.618, 3.1416] j2 [0, 3.6652] j3 [0, 3.1416] j4 [-1.693, 1.5708]
  j5 [-1.5708, 1.5708] j6 [-2.0944, 2.0944]。
- FK(q=0)：xyz (0.1106, 0, 0.1733)，quat(wxyz) (0.5, -0.5, 0.5, -0.5)。
- IK（mink + quadprog，FrameTask on body gripper）：30/30 收敛，平均 0.41 ms、最大 1.39 ms，
  位置误差 <0.1 mm（30 Hz 预算 33 ms，余量充足）。

## 机器人侧（i2rt）

- `get_yam_robot(channel, arm_type=ArmType.YAM_PRO, gripper_type=..., sim=False)`；
  关节接口 `get_joint_pos()/command_joint_pos()`，7 维（6 臂 + 1 夹爪，夹爪归一化 0=闭 1=开）。
- `sim=True` → `SimRobot`（MuJoCo），接口一致，**可用于无硬件联调**。
- 实测两台 YAM Pro：右臂 can_follower_r、左臂 can_follower_l（当前被遥操采集占用）。

## 相机（远端实测）

- 2 台 RealSense D405：serial 262422270050（左腕）、262422272588（右腕）。
- bimanual 配置里的主视角是 **D435 serial 327122079322**，但当前 **未枚举到**（可能没接）。
- D405 USB 描述符有 3 个 Video Streaming 接口（0x8086:0b5b）；但 librealsense 2.59.1 静态枚举只报
  `Stereo Module`（设备正被采集进程占用时可能不完整）。他们的 gello 代码确实以
  `rs.stream.color 640x480 bgr8 @30` 打开 D405 且在生产中使用 → 彩色流应可用，**需在设备空闲时实测确认**。
- 本环境已装 `pyrealsense2==2.59.1`（他们环境是 2.58.3）。
