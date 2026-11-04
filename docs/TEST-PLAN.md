# 上机测试计划（2026-11-03 18:00）

## 目标
1. 真机打通：预览 -> 标定 -> 启用控制 -> 任务闭环
2. 三个任务各 20 次，记录成功率：pick_place（抓拿放）、pour_water（倒水）、stack_cups（叠杯子）

## 阶段 0：上机前（5 分钟）
- 确认遥操/采集进程已停，CAN 空闲: ip -br link | grep can; pgrep -af 'collect_bimanual|yam_safety'
- 确认实际是右臂 can_follower_r 还是左臂 can_follower_l；改 config-real.yaml 的 channel + gripper_limits
- 确认夹爪型号（linear_4310 vs 3507 vs crank）；错了在标定前改
- 确认腕部相机 serial 与腕部一致（左 262422270050 / 右 262422272588）
- 磁盘余量: df -h /home（录像会占空间）
- 启动: roboapi http -c config-real.yaml
- roboapi state -c config-real.yaml 应显示 arm_connected + 相机帧龄正常

## 阶段 1：标定（人工配合，10 分钟）
1. 控制台 -> 设置 -> 设备 -> 坐标标定 -> 开始
2. 此时臂进入重力补偿可手推状态；用双手扶住，缓慢摆到 Home（夹爪竖直朝下，台面上方 30-50 cm）后记录
3. A：保持姿态，用夹爪指尖接触台面（同一接触点）后记录
4. B：沿相机画面左方沿台面移动（A->B 定义任务 +X）后记录
5. C：同一台面不与 A/B 共线的点，记录后臂自动回 Home
6. 方向试动：向前/向左/向上各一次，确认位移与旋转方向符合页面示意
7. 启用控制；确认 Robot Stop 有效（先做一次停止测试）

## 阶段 2：三个任务各 20 次
- 每次: 在控制台输入指令 -> 发送 -> 观察 -> 判定 success/fail -> 记录
- 记录命令: .venv/bin/python tools/trial_log.py --task pick_place --trial 1 --result success --note '...'
- 汇总: .venv/bin/python tools/trial_log.py --summary
- 每次开始前确保臂在 Home/安全位置；连续失败 3 次先停下来查原因，不要凑数
- 任务之间清理台面、恢复物体初始摆放
- 每次记录失败模式（抓空/掉落/超时/碰撞/模型乱动），这比成功率本身更有信息量

## 成功判据（待现场确认）
- pick_place: 物体被抓起并放置到目标容器内，最终稳定不滑落
- pour_water: 抓起容器、倾倒动作发生、水进入目标杯且桌面无明显大量外洒
- stack_cups: 上方杯子稳定放在下方杯子上，松手后不掉落

## 安全
- 人手不要进工作区；急停优先按 Robot Stop
- 首次连接会触发 i2rt 启动自检（可能写电机 CTRL_MODE 等寄存器），属正常行为
- 若臂出现异常抖动/失控: Ctrl+C 停控制台；电机 400 ms 无指令会进阻尼
