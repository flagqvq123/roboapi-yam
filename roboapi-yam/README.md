# roboapi-yam — RoboAPI 的 YAM 硬件适配器

把 **YAM Pro 机械臂 + 平行夹爪 + 腕部相机** 接入 RoboAPI 0.4.5 的本地控制台：页面里可以预览、标定、启用控制、跑策略任务，也能通过本地 HTTP API 被脚本驱动。

## 1. 架构

```
roboapi http (RoboAPI 控制台)
  └─ roboapi-yam（本包，entry points 注册 arm / gripper / camera）
      ├─ backend.py   SimBackend(MuJoCo) / CanBackend(SocketCAN)，只负责设备 I/O
      ├─ session.py   每个 (mode, channel, arm_type, gripper_type) 一个会话：
      │               唯一设备句柄 + 100 Hz 控制线程 + FK/IK + free/servo 模式
      ├─ kin.py       6 自由度法兰 FK/IK（MuJoCo + mink）
      ├─ arm.py       Arm / FeedbackReader
      ├─ gripper.py   0=闭 1=开，与臂共用会话
      └─ camera.py    RealSense 彩色流（synthetic 合成源用于干跑）
```

机械臂/夹爪/反馈都是同一会话的视图，保证一条 CAN 只有一个持有者。

## 2. 环境需求

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 x86-64 |
| Python | 3.11（roboapi 要求 3.10–3.12，i2rt 建议 3.11） |
| roboapi | 0.4.5（PyPI） |
| i2rt | 1.1.2（源码 editable 安装），带动 mujoco / mink / python-can |
| CAN | SocketCAN，1 Mbit：左臂 `can_follower_l`，右臂 `can_follower_r` |
| 相机 | Intel RealSense D405（`pyrealsense2`）；可选 Hikrobot MVS SDK（`/opt/MVS`） |
| 系统库 | `libgl1`、`libglib2.0-0`（opencv 需要） |

## 3. 安装

```bash
mkdir -p ~/roboapi && cd ~/roboapi
uv venv --python 3.11 .venv && source .venv/bin/activate

uv pip install 'roboapi==0.4.5'                       # 控制台
uv pip install -e ./vendor/i2rt                        # YAM 驱动（源码）
uv pip install -e ./roboapi-yam                        # 本适配器
uv pip install pyrealsense2                           # 相机（用 RealSense 时）
```

> i2rt 的 ruckig 在新版 scikit-build 下编译失败，安装时加
> `-b <(echo 'scikit-build-core<0.10')`（build constraint）。

## 4. 配置

复制一份并改三个地方：`channel`、`camera.serial`、`policy`。

```yaml
arm:
  type: yam
  mode: real                 # real=真机, sim=MuJoCo
  channel: can_follower_l    # 左臂；右臂 can_follower_r
  arm_type: yam_pro
  gripper_type: linear_4310
  control_hz: 100.0
  max_joint_speed: 2.0
gripper:
  type: yam
  mode: real
  channel: can_follower_l
  gripper_type: linear_4310
camera:
  type: yam
  source: realsense          # 或 synthetic（无相机干跑）
  serial: '262422271903'     # D405 序列号
policy:
  type: openai
  url: https://api.pokebot.com/v1
  model: poke-aurora-preview
  use_env_proxy: false       # 环境里若有 SOCKS 代理，关掉更稳
  token_file: ~/.config/roboapi-yam/token
```

夹爪行程默认**自动探测**（0.2 N·m 慢速扫两端）。已知实测值时可在 `gripper.gripper_limits: [closed, open]` 指定以跳过探测。

## 5. 启动与停止

```bash
cd ~/roboapi
./.venv/bin/roboapi http -c config-real.yaml      # 前台；Ctrl+C 退出
```

- 启动后是**预览**：臂进入重力补偿（软、可手推），不会自己动；
- 页面点“启用控制”才进入位置伺服，可以跑任务；
- 入口 `http://127.0.0.1:8080/`（远程用 `ssh -L 8080:127.0.0.1:8080 ps@<host>`）。

运行中随时切换（不需要重启）：

```bash
python scripts/yam_mode.py gravity   # 立刻变软，可手推摆位
python scripts/yam_mode.py hold      # 锁在原地
python scripts/yam_mode.py servo     # 交回伺服（等待指令）
```

**退出前注意**：进程结束（Ctrl+C / pkill）时 i2rt 会关链并把力矩置零，臂会失去支撑。先 `yam_mode.py gravity` 并扶住或放低，再退出。

## 6. 标定

页面 → 设置 → 设备 → 坐标标定：

1. 手推臂到 Home（夹爪朝下、台面上方 30–50 cm）→ 记录；
2. A：指尖轻触台面（同一接触点）→ 记录；
3. B：沿画面左方向在台面移动（A→B 定义任务 +X）→ 记录；
4. C：台面上不与 A/B 共线的点 → 记录，臂自动回 Home；
5. 方向试动确认 5 cm / 30° 方向正确 → 启用控制。

标定 identity 绑定 **CAN 通道 + 相机序列号**，换臂或换相机会自动失效、需要重标。

## 7. 跑试验

```bash
./scripts/run_trials.sh --task stack_cups --trials 20
```

每轮：启动一次运行 → 跟随动作块直到触发预算 → 停止 → 回 Home → 询问结果/评分/备注 → 等你按 Enter 还原场景。

| 参数 | 默认 | 含义 |
|---|---|---|
| `--max-seconds` | 60 | 每轮墙钟上限 |
| `--stall-chunks` / `--stall-mm` | 3 / 5 | 连续 3 块净位移 < 5 mm 判定停滞 |
| `--chunks` | 0（不限） | 块数上限 |
| `--trials` | 20 | 轮数 |

结果写 `~/roboapi/trials/<task>_<时间>.csv`，列含 `task_id / chunks_run / total_displacement_m / stop_kind / result / rating`。退出时自动切重力并打印成功率。

## 8. 录像与回看

录像在 `logs/tasks/<task_id>/video.mp4`（附带 `metadata.json`）。可读名索引：

```bash
./scripts/make_review.sh        # 生成 ~/roboapi/review/*.mp4 软链接
```

命名 `日期_时间_任务_t轮次_结果.mp4`，指向真实文件，不影响控制台。

## 9. 无硬件干跑

```bash
# 1) 本地 stub 策略服务
.venv/bin/python tools/policy_stub.py --port 9010 &
# 2) 配置里 policy.url 指向 http://127.0.0.1:9010/v1，arm.mode=sim
./.venv/bin/roboapi http -c config-sim.yaml
```

## 10. 已知限制

- **模型协议没有“任务完成”标记**：只能按预算/停滞/人工停止，成功判定靠人；
- 单目腕部相机 + 与训练参考不同的相机几何（FOV/畸变/安装位姿），策略效果不保证；
- 真机“只读反馈”实际是重力补偿空闲态（i2rt 对 DM 电机没有纯被动读位置）；
- 进程退出会关链卸力（i2rt 语义），不是断电但会失去支撑。

## 11. 故障排查

| 现象 | 处理 |
|---|---|
| 启用控制报 `socksio` 缺失 | `uv pip install "httpx[socks]"`，或配置 `policy.use_env_proxy: false` |
| 夹爪电机发烫/过热停链 | 行程参数错导致堵转：删掉 `gripper_limits` 让 i2rt 自动探测 |
| `at_home=false`、方向试动被拒 | Home 容差 5 mm/2°，用页面“初始位置”重试 |
| `motor ... error: disabled` | 链被前一次异常停过，重启控制台重新初始化 |
| 回放找不到 | 先 `./scripts/make_review.sh`，或直接看 `logs/tasks/<task_id>/` |

## 12. 诊断脚本

`scripts/diagnostics/` 下有：FK/IK 精度检查、适配器冒烟、free/servo 切换、相机代码路径、Hik 相机抓帧、哪台相机在动臂上等工具，均为只读或独立进程。

---

版本：roboapi-yam 0.1.0 · 对应 RoboAPI 0.4.5 · 记录于 2026-11-04
