# model_deploy：双臂 ACT 部署与轨迹回放

四路独立 CAN 的 ROS 主从入口为 `~/AgileX_ws/teleop.sh`，完整操作、速度参数和采集方式见[主从控制说明](../../docs/teleop.md)。
已确认映射：左主 can1、右主 can2、左从 can3、右从 can0。遥操作、ACT、复位和回放共享控制锁，不能同时控制从臂。

[返回工作空间 README](../../README.md) · [硬件模式切换](../../docs/hardware.md#切换部署模式与恢复采集) · [训练与检查点](../../training/README.md)

`start_hardware.sh` 发布相机/机械臂实际反馈；`deploy.sh` 订阅 ROS，调用现有 LeRobot ACT，经过动作检查后用 pyAgxArm SDK 控制左右从臂和夹爪。ROS 使用系统 Python 3.8，GPU 推理使用 `lerobot` Python 3.12，两者由脚本自动管理。

## 单臂或双臂

默认双臂。部署/离线推理支持 `--arms both|left|right`，可用 `--checkpoint PATH` 覆盖 YAML 中的权重路径。
例如 `deploy.sh --arms right --checkpoint /path/to/right/pretrained_model --dry-run`。
硬件需对应使用 `start_hardware.sh mode:=deploy arms:=right`，执行时只连接该侧 CAN 和反馈。
单臂严格要求同侧的 7 维 ACT 检查点，不能从当前 14 维双臂模型中自动截取；三相机要求保留。
输出 JointState 的 name/position 与所选侧一致，为 7 维；不选择时仍为 14 维。

`reset.sh --arms left/right` 只复位所选从臂；`teleop.sh --arms left/right` 只启动该侧主从节点，
采集终端 `r` 只复位所选主从臂。复位位置和限速仍使用原有配置。
参数也可通过 `act.yaml`、`teleop.yaml` 的 `active_arms` 持久设置；命令行选择优先。
双臂行为和 `replay.sh` 的既有双臂回放入口保留。完整操作见[工作空间说明](../../README.md#选择双臂或单臂)。

## 先运行只推理模式

普通部署仅需双从臂和相机通电，主臂可断电。先退出采集遥操作与旧硬件；终端 1：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy
```

终端 2：

```bash
~/AgileX_ws/deploy.sh --dry-run
```

默认运行 60 秒，也可加 `--duration=10`。不指定运行模式也默认为 dry-run。该模式不创建 SDK 控制器、不打开 CAN、不发送使能或运动指令；动作检查结果、耗时和预测值保存在日志中。

## 正式执行

先退出 `teleop.sh` 及其他 SDK 控制程序，释放从臂控制锁；主臂可以断电。手动将从臂及物体放到采集时的起始状态。`prepare_can.sh --mode=deploy` 默认仅准备从臂 can3/can0。

停止旧的硬件发布和部署进程，再切换 CAN 模式（sudo 密码只在本机终端输入）：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
```

终端 1：

```bash
~/AgileX_ws/start_hardware.sh mode:=deploy
```

首次可先查询固件，不使能、不移动：

```bash
~/AgileX_ws/deploy.sh --probe-firmware
```

查询会发送固件请求报文，需要正常收发 CAN；失败时不会继续使能。执行模式也会查询固件、选择 SDK profile，并在运行目录保存 `firmware.json`。可将确认后的软件版本填入 YAML `expected_firmware`，以后遇到版本变化就拒绝执行。

终端 2，先验证实时推理，再执行：

```bash
~/AgileX_ws/deploy.sh --dry-run --duration=10
~/AgileX_ws/deploy.sh --execute
```

`--execute` 在输入和模型检查通过后等待回车。按回车后监听外部控制报文、查询固件、重新检查最新姿态，然后以当前位置替换旧目标并使能，开始运行。无需再开一个 SDK 终端。默认最多运行 60 秒；模型没有“插接成功”检测器，由操作者决定提前停止。

按 **Ctrl+C** 或到达时间限制，会请求双臂保持最新实测姿态、夹爪保持最新实测开口，不回零、不松爪、不关闭电机。反馈已过期时无法可靠请求当前位置保持，改为请求 SDK 阻尼停止。CAN 断开、进程被强制杀死或控制器不响应时，软件不能保证机械保持；首次必须实机验证停止响应，不能把退出 Python 当作急停。

恢复采集前，先停止部署和硬件，再运行：

```bash
~/AgileX_ws/prepare_can.sh --mode=teleop
~/AgileX_ws/teleop.sh --execute --cameras=all
# 输入 d，夹爪接管且 READY 后再启动 record.sh。
```

部署模式不发布 `leader_targets`、`gripper/target_stamped`，避免把电脑控制回显当成主臂示范；回显仍可在 `events` 中追溯，并标记来源。采集模式维持原有话题。

## 复位到固定姿态

普通模型退出后，保持 `start_hardware.sh mode:=deploy` 运行，在第二终端执行。这里只连接、复位两台从臂，主臂可以断电：

```bash
~/AgileX_ws/reset.sh --dry-run   # 可选：只打印目标与动作顺序，不连接 SDK/CAN
~/AgileX_ws/reset.sh
```

依次执行双侧夹爪打开到 100 mm → 允许末端朝向变化、沿关节路径抬高 10 cm → 回到固定 reset 关节姿态 → 到位后闭合。用户已确认两侧底座水平安装。每个阶段检查实测位置连续到位后才进入下一阶段；抬升失败不继续回位，回位未完成不闭合。

默认 `reset.vertical_lift.mode: joint_path`：使能和运动前读取两侧设备关节限位，与配置范围取交集；核对 SDK 正运动学与实测法兰姿态，再用仅约束位置的逆解规划整段抬升。关节限位内预留 0.03 rad 余量，起点已靠近限位时不再向限位外侧推进。规划优先保持 X/Y，允许最多 5 cm 水平偏移、0.8 rad 朝向变化；轨迹和关节插值中间点须通过高度、限位及偏移检查，否则运动前退出。规划结果保存为 `lift_path.json`。这些检查不包含碰撞检测。

抬升通过 SDK `move_j` 逐步执行，法兰目标推进速度上限 2 cm/s，同时受复位关节速度、control 限速和跟踪约束。只有双侧法兰的实测高度均增加 `distance_m`（默认 0.10 m，容差 3 mm）并稳定 0.5 秒，才开始回位；向下偏移、超高或法兰跟踪误差超过 1 cm 时停止，各阶段超时 30 秒。`distance_m` 是从本次抬升起点向上的相对距离，反复执行会从每次当前位置重新抬升。由于允许朝向改变，该高度指法兰，不保证夹爪尖端或夹持物各点也抬高同样距离。旧的固定朝向直线方式可通过 `mode: linear` 选择，使用 `orientation_tolerance_rad` 检查朝向。

抬升完成后以新鲜关节实测值开始关节插值回位。打开会释放夹持物，请先取下或托住物体，并解除插接接触、留出空间。抬升不能保证整条臂身和后续回位段不碰桌面；当前没有桌面模型、碰撞检测或避障规划，最终 reset 姿态及其路径也须留有间隙。`--dry-run` 仅打印相对抬升要求，不读取设备限位或验证当前姿态的可达性。

默认姿态在 `act.yaml` 的 `reset.home_joints_rad`，左右分别按 J1~J6 填写，单位 rad。当前左右均为用户指定的 `[0, 0, 0, 0, 24, 0]°`，以弧度存储；后续修改以配置文件为准。它不随新一次部署改变，也不依赖原日志文件。模型仍在运行时，复位会因 CAN 控制锁被占用而拒绝启动。

采集遥操作运行时在 `teleop>` 输入 `r`，由已有四个节点完成从臂复位与主臂对齐，不另开 `reset.sh`，详见[采集复位](../../docs/teleop.md#启动)。RL 人在环尚未实现；当前两个入口不提供模型与人工控制权切换。

如需临时改用某次部署的起始姿态，可以显式传入 `--from-run` 覆盖固定姿态。新版日志读取 `start_pose.json`；旧版日志使用首帧观测作为近似起始姿态：

```bash
~/AgileX_ws/reset.sh --from-run ~/AgileX_ws/logs/deploy/20261016_181029_984172 --dry-run
~/AgileX_ws/reset.sh --from-run ~/AgileX_ws/logs/deploy/20261016_181029_984172
```

`--execute` 仍保存起始快照供追溯及显式选择；默认 `reset.sh` 不使用它。复位不加载模型，不要求相机/GPU，但需要正常 CAN、双臂新鲜反馈及部署模式的硬件状态。配置位于 `act.yaml` 的 `reset` 段；默认关节路径抬升与关节回位均受 `control` 的逐步限速、跟踪和物理范围检查约束。旧 `linear` 模式使用单独的平移速度/路径偏差检查及固件直线插补。所有模式均检查实际关节范围、机械臂状态和通信超时。

Ctrl+C、反馈失效或任一阶段超时都会结束复位并调用同部署程序的停止处理；不会跳过失败阶段。进度发布到 `/agilex/reset/status`，日志保存在 `logs/deploy/reset_<时间>/`。这是姿态回位，不调用 SDK 回零、零位标定或电机断使能。

## 从 bag 回放实机轨迹

`replay.sh` 读取 bag 中左右从臂 `joint_states` 和 `gripper/actual_stamped`，按 `JointState.name` 取 J1~J6、夹爪开口。使用实际走过的轨迹，不使用 `leader_targets`，不加载 ACT，也不要求相机或 GPU。

```bash
# 离线检查完整 bag，显示起点（角度/毫米）、时间范围和边界修正
~/AgileX_ws/replay.sh ~/AgileX_ws/bags/insert_plug_v2/task_0001 --dry-run
# 也可以直接传入 data.bag；不加模式参数默认同样只预览
```

正式执行需要主臂退出控制、双从臂已手动放到录制起点。停止旧的硬件启动及控制进程后，在终端 1 准备 CAN 和启动反馈：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy cameras:=none
```

终端 2 执行（检查通过后等待回车）：

```bash
~/AgileX_ws/replay.sh ~/AgileX_ws/bags/insert_plug_v2/task_0001 --execute
# 原速：追加 --rate 1；默认 --rate 0.5
```

如果需要自动准备起点，使用：

```bash
~/AgileX_ws/replay.sh ~/AgileX_ws/bags/insert_plug_v2/task_0001 --execute --move-to-start
```

按回车后从最新实测姿态开始，对双臂关节与夹爪做同步关节空间插值，到位并稳定后自动进入回放。默认接近速度上限 0.2 rad/s、夹爪 0.01 m/s，整个阶段超时 30 秒；仍受控制器限速、故障和反馈检查约束，到位失败不回放。参数为 `approach_joint_speed_rad_s`、`approach_gripper_speed_m_s`、`approach_timeout_sec`。该步骤不抬臂、不避障，需要解除接触并确保从当前姿态到起点的路径畅通。与 `--dry-run` 同用时只显示启用此步骤，不读取实机或移动。

配置 [replay.yaml](config/replay.yaml) 独立设置速度和回放检查；CAN、固件、夹持力度、物理范围、反馈时效及控制跟踪约束沿用 [act.yaml](config/act.yaml)。可分别通过 `--replay-config`、`--config` 覆盖。默认 50 Hz 下发、SDK 速度比例 20%、关节推进上限 0.6 rad/s、夹爪 0.03 m/s，不改动模型部署的速度参数。

轨迹处理与执行规则：

- 四路状态各至少两条有效消息，以 header 时间取共同覆盖区间，线性插值，不对关节角做周期折返。原始时间端点及裁剪长度写入 `plan.json`；不是逐条发送 bag 中约 200 Hz 的全部原始消息。
- 拒绝不完整录制、时间倒退、无效数值及大于 100 ms 的状态缺口。超出物理范围但处于 `act.yaml` 反馈容差内的测量值，投影到范围边界，并显示修正数量和最大修正值；更大越界直接拒绝。
- 默认要求每关节起点偏差不超过 0.15 rad（约 8.6°）、夹爪不超过 15 mm，按回车后、使能前再次检查。显式加 `--move-to-start` 时先移动到起点，稳定到位后再做起点检查；不调用 reset 的开爪或抬臂动作，夹爪随关节调整到录制开口。
- 真实关节、夹爪和故障状态来自 SDK；ROS 仅用于确认硬件启动模式与 CAN 映射。禁止仿真时钟，继续使用控制互斥锁、外部主臂指令检测和通信看门狗。
- 每个重采样路点需发送目标到位且实测误差不超过 0.04 rad / 4 mm，才推进下一点；落后时等待，不跳过路点追赶。默认路点超时 10 秒，终点连续到位 0.5 秒后结束。因此 `--rate` 是目标倍率，限速、SDK 规划、跟随等待和终点确认可能增加总时长。
- 正常完成请求保持最终实测姿态和开口；Ctrl+C、跟随超时、反馈失效走同部署的停止处理。它不自动调用 reset，不回零、不松爪。夹持物或插接条件与录制不同，可能无法到位；此模式没有力控、桌面碰撞检测或避障。

不要同时运行 `rosbag play`、`deploy.sh --execute`、`reset.sh` 或其他 SDK 控制程序。日志位于 `logs/replay/<时间>/`：配置、轨迹计划、固件、`trajectory.npz`、逐步目标/发送值/实测值 `actions.jsonl` 和执行结果 `result.json`。进度也发布到 `/agilex/replay/status`。

## 模型与配置

默认配置：[config/act.yaml](config/act.yaml)。指定其他配置：

```bash
~/AgileX_ws/deploy.sh --config=~/AgileX_ws/src/model_deploy/config/act.yaml --dry-run
```

加载的检查点以 [act.yaml](config/act.yaml) 的 `checkpoint` 为准。该字段需显式选择，训练新模型后不会自动切换；切换前核对训练批次、输入特征、14 维顺序及动作定义。v3 全量训练未划分验证集，不能将其最新检查点描述为“验证损失最低”。

部署加载同目录的模型、归一化和反归一化文件，禁止联网补权重。当前实现要求三路 RGB＋14 维绝对关节/夹爪位置的数据约定，不能仅凭维数相同就替换成另一种动作定义的模型。

| 内容 | 约定 |
| --- | --- |
| 图像输入 | `/{cam_h,cam_l,cam_r}/color/image_raw`，RGB，640×480 |
| 状态输入 | `/piper/{left,right}_bus/joint_states`、`gripper/actual_stamped` |
| 14 维顺序 | 左六关节、左夹爪、右六关节、右夹爪 |
| 动作 | 同顺序的绝对关节角 rad、夹爪开口 m |
| 运行节拍 | 15 Hz，新观测推理一次；ACT 时序融合系数 0.01，每次执行当前一步 |
| 控制接口 | SDK `move_j` + `move_gripper_m`；不使用快速无规划 `move_js` 或力矩控制 |
| 深度/电流/力矩 | 本次模型未使用；运行时监视从臂状态错误码，不把估算力矩当成碰撞保护 |

左右按 `JointState.name` 重排。选择最新完整的相机组合，再匹配不晚于参考图像的关节反馈；默认相机偏差最多 50 ms、关节对齐误差最多 100 ms。单独限制图像/反馈的新鲜度和本机接收时延。没有跨相机硬件同步。

限位按标准 Piper 设置，第六关节 ±180°、夹爪最大 100 mm 由用户在 2026-10-16 确认；同时传入 SDK，覆盖本机 SDK 的第六关节默认 ±120°。SDK 速度比例和程序逐步限速同时作用，具体数值以当前 `act.yaml` 为准。

当前 `check_initial_target_error: false`、`check_runtime_target_error: false`，关闭启动和运行时“原始模型目标离实测位置过远就停止”的检查；对应的 `max_initial_error_*`、`max_target_error_*` 仅在开关改为 `true` 时生效。旧配置缺少开关时默认仍开启检查。原始预测仍要求有限数值，发送前投影到物理范围并逐步限速；关闭检查不会把远处目标直接一次性下发。使能握手期间的异常位移检查仍使用 `max_initial_error_*`，不受上述开关影响。

关节实际指令跟踪检查继续使用 `max_tracking_error_rad`：比较上一次已下发目标与当前实测位置，超出则停止；报错包含具体关节、已下发目标、实测值、偏差、阈值和单位。当前 `check_gripper_tracking_error: false`，双侧夹爪不会仅因跟踪偏差超限而退出；改为 `true` 或旧配置缺少此项时仍检查。`max_tracking_error_m` 继续约束夹爪下发目标相对实测开口的范围，避免夹住物体后闭合指令不断累积；反馈突然跳变时按限速逐步回到该范围，不直接跳变目标。夹爪夹持力度参数未改变。

反馈越界、反馈过期和通信故障停止逻辑继续保留。启动终端打印检查开关状态，修改配置后重新运行脚本生效。此夹爪设置也用于复位控制器，但复位每个阶段仍须反馈到位，否则超时退出。

这些是目标变化约束，不是力控或笛卡尔碰撞检测。当前参数按用户要求适度放宽，尚需观察跟随误差和实际插接效果。`gripper_force` 是 SDK 协议参数，未做夹持力标定。

控制端有独立于推理进程的 20 ms 检查循环，发现数据过期、命令超时、SDK 错误或外部主臂命令后锁定停止状态，不自动恢复。文件锁防止本工具多实例同时控制同一 CAN；它不能约束其他独立 SDK 程序，请关闭它们。

## 查看输出

```bash
source ~/AgileX_ws/env.sh
rostopic echo /agilex/deploy/status
rostopic echo /agilex/deploy/predicted_action
rostopic echo /agilex/deploy/commanded_action
```

后两个话题均为 `sensor_msgs/JointState`，`position` 在双臂模式为 14 维、单臂模式为 7 维，`name` 给出所选侧和顺序。预测话题时间是参考观测时间，下发话题时间是发送时间。dry-run 不向 `commanded_action` 发布消息。

每次运行新建 `logs/deploy/<时间>/`，包含 `config.json`、`inference.log`、`actions.jsonl`、`result.json`；控制/固件查询另外保存 `firmware.json`。逐步日志保留源时间、状态、预测、限幅维度、实际发送目标和耗时，不保存重复图像。日志中的“已发送”不代表物理执行已确认。

## 无硬件验证

```bash
~/AgileX_ws/deploy.sh --offline ~/AgileX_ws/datasets/piper_insert_plug --frames=30
source ~/AgileX_ws/env.sh
/usr/bin/python3 -m unittest discover -s ~/AgileX_ws/src/model_deploy/tests -p 'test_*.py' -v
```

`--offline` 不启动 ROS、不加载 SDK。完整 ROS 集成测试使用独立 master 和录制数据，检查正常运行、反馈中断、推理退出及 Ctrl+C：

```bash
~/AgileX_ws/deploy.sh --offline ~/AgileX_ws/datasets/piper_insert_plug --frames=1 --fixture=/tmp/agilex-deploy-fixture.npz
source ~/AgileX_ws/env.sh
/usr/bin/python3 ~/AgileX_ws/src/model_deploy/tests/ros_smoke.py --fixture=/tmp/agilex-deploy-fixture.npz
```

已完成软件测试不等于完成实机部署验收。固件匹配、实际停止效果和插接成功率需要实机验证。

历史测试结果、固件查询与复位验证范围见[工作空间验证记录](../../docs/validation.md#act-部署固件查询与复位2026-10-16)。当前配置与当时测试条件可能不同，历史结果不代表当前配置已完成实机验收。
