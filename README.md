# RoboAPI × YAM

RoboAPI 0.4.5 的 YAM（YAM Pro）硬件适配器：把机械臂、平行夹爪和腕部相机接入 RoboAPI 本地控制台，
支持预览、坐标标定、启用控制、跑策略任务，以及一套批量试验脚本。

## 目录

| 路径 | 内容 |
|---|---|
| `roboapi-yam/` | 适配器包（entry points 注册 `arm / gripper / camera`） |
| `roboapi-yam/README.md` | **技术文档**：架构、环境需求、安装、配置、标定、试验、排错 |
| `scripts/` | 运行脚本：`trial_runner.py`、`yam_mode.py`、`link_tasks.py` 等 |
| `scripts/diagnostics/` | 只读诊断：FK/IK、适配器冒烟、相机识别 |
| `docs/` | RUNBOOK / TEST-PLAN / NOTES / ENV-SUMMARY |
| `configs/` | 真机与仿真配置示例 |
| `tools/policy_stub.py` | 本地 stub 策略服务（无云端 API 也能干跑） |
| `dist/` | 构建产物（wheel + sdist） |

## 最快上手

```bash
# 1) 装 RoboAPI 与 i2rt（需要 Python 3.11）
uv pip install 'roboapi==0.4.5'
uv pip install -e /path/to/i2rt

# 2) 装本适配器
uv pip install dist/roboapi_yam-0.1.0-py3-none-any.whl

# 3) 配置（改 channel / camera.serial / policy）
cp configs/config-real.example.yaml ~/roboapi-config.yaml

# 4) 启动控制台
roboapi http -c ~/roboapi-config.yaml      # http://127.0.0.1:8080/
```

启动后是**预览**（重力补偿、可手推）；在页面点“启用控制”并完成 Home + A/B/C 标定后才能跑任务。
详见 [`roboapi-yam/README.md`](roboapi-yam/README.md)。

## 关键设计

- 每个 `(mode, channel, arm_type, gripper_type)` 只有一个会话：唯一 CAN 句柄 + 100 Hz 控制线程；
  臂、夹爪、只读反馈都是这个会话的视图，避免多进程抢总线。
- 笛卡尔目标 → IK（MuJoCo + mink，约 0.4 ms/次）→ 关节指令；带重力下垂补偿（实测回 Home 误差约 4 mm）。
- 断开时不会关链卸力：最后一个使用者释放后转入原地保持，只有在进程退出时才关闭。

## 已知限制

- 模型协议没有“任务完成”标记，结束条件只能靠预算 / 停滞检测 / 人工停止；
- 相机与训练参考不同（FOV、畸变、安装位姿），策略效果不保证；
- 真机只读反馈实际是重力补偿空闲态（i2rt 对 DM 电机没有纯被动读位置）；
- 进程退出时 i2rt 会关链置零力矩，臂会失去支撑，收工前先切重力并扶好。

## 许可

本项目使用 [PolyForm Noncommercial License 1.0.0](LICENSE)：允许非商业用途的使用、修改与分发，
商业用途需另行获得许可。
