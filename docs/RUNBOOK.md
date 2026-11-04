# RoboAPI × YAM 适配 — 运行手册（2026-11-03 更新：真机前准备完成）

## 目录

    ~/roboapi/
      .venv/                  Python 3.11.16 + roboapi 0.4.5 + i2rt 1.1.2（editable）
      vendor/i2rt/            i2rt 源码
      roboapi-yam/            适配器包（editable 安装到同一 venv）
      tools/policy_stub.py    本地 stub 策略服务
      scripts/                探针、检查、标定生成脚本
      config-sim.yaml         仿真配置（可复制为 config-<机器名>.yaml）
      logs/

## 适配器结构

    src/roboapi_yam/
      backend.py   SimBackend(MuJoCo) / CanBackend(SocketCAN); free_mode() = 重力补偿可手推
      session.py   每个 (mode, channel, arm_type, gripper_type) 一个会话:
                   唯一设备句柄 + 100 Hz 控制线程 + FK/IK; free=True 时只读不控
      kin.py       6 自由度法兰 FK/IK（模型由 arm_type 解析，无绝对路径）
      arm.py       Arm / FeedbackReader; Feedback 在真机用 free 模式（重力补偿）读状态
      gripper.py   夹爪 0=闭 1=开，共享会话
      camera.py    synthetic / realsense（rgb8 -> bgr8 -> 640x480 -> 红外 逐级回退，等比缩放到 1280x720）

## 真机（real 模式）行为

- 反馈: connect 会话进入 free 模式 -> i2rt 重力补偿空闲，操作员可手推示教;
  不发送位置目标，因此读状态不夺控制权
- 控制: arm.connect() 把同一会话切回 servo，30 Hz 笛卡尔目标 -> 100 Hz IK -> 关节指令
- 设备身份（写进标定 profile）: {backend: can, channel: <can口>, arm_type: ..., gripper_type: ...}
  换 CAN 口或换臂型都会要求重新标定
- 启动前检查: real 模式下 check_motion_ready() 检查 /sys/class/net/<channel> 是否存在

## 常用命令

    source ~/roboapi/.venv/bin/activate
    roboapi state -c config-sim.yaml
    roboapi http  -c config-sim.yaml            # http://127.0.0.1:8080/
    .venv/bin/python tools/policy_stub.py --port 9010

    python scripts/check_kin.py                 # FK/IK 精度与耗时
    python scripts/check_adapter.py             # 臂/夹爪/相机 冒烟
    python scripts/check_feedback.py            # free 模式与 servo 切换
    python scripts/check_realsense_stub.py      # 相机 RealSense 代码路径（假设备）

## 真机上线清单（等 YAM 空闲后）

1. 复制 config-sim.yaml -> config-real.yaml，改: arm/gripper 的 mode: real、channel
   （右臂 can_follower_r / 左臂 can_follower_l），camera.source 改 realsense（或保留 synthetic）
2. 确认没有别的进程占用 CAN: ip -br link | grep can、ss -ltnp | grep -E "6021|6022";
   安全前提是 i2rt 的 400 ms 电机超时机制，拔线/崩溃会自动进阻尼
3. roboapi state -c config-real.yaml: 应显示 arm_connected、相机帧龄正常
4. 控制台里重新做坐标标定（真机必须人工采 Home+A/B/C; 仿真那份 profile 的 identity 不同）
   标定期间臂处于重力补偿，可直接手推到位
5. 启用控制 -> 方向试动（5 cm/30 度）-> 小范围任务; 先确认 Robot Stop 有效
6. 若相机报错: 检查是否被遥操进程占用; 占用时先用 synthetic 跑通其余链路

## 已知限制

- CanBackend 与真机反馈尚未在实机验证（本轮无设备）
- D405 彩色流未实测; RealSense 代码路径仅用假模块验证（640x480 回退 + 缩放已覆盖）
- 真机夹爪零位/行程依赖 i2rt 的 gripper_limits; 不同夹爪型号需在 config 里给
  arm.gripper_limits: [closed, open]
