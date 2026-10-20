# AgileX_ws：双组 Piper 主从臂与三台 D435

ROS 1 Noetic 工作空间，用于四路独立 CAN 的 Piper ROS 主从控制、硬件状态发布、rosbag 采集、LeRobot 数据转换、ACT 训练及双从臂部署。当前四台臂各接一个 USB-CAN，主从之间通过 ROS 和 SDK 软件跟随。

完整流程：**准备环境 → 启动硬件 → 录制演示 → 转换数据 → 训练 ACT → 部署验证**。

- [环境准备](#环境准备)
- [硬件与运行模式](#硬件与运行模式)
- [数据采集](#数据采集)
- [数据转换与-act-训练](#数据转换与-act-训练)
- [模型部署与实机操作](#模型部署与实机操作)
- [配置与详细文档](#配置与详细文档)

## 环境准备

依赖 ROS Noetic、RealSense C++ SDK、python-can 和 pyAgxArm 1.0.0。ROS 使用系统 Python 3.8；LeRobot 转换、训练及推理使用独立 Python 3.12 环境，本机为 Conda `lerobot`。入口脚本自动加载或隔离相应环境。

直接运行 ROS 命令前加载工作空间；修改代码后编译、检查：

```bash
source ~/AgileX_ws/env.sh
~/AgileX_ws/build.sh
~/AgileX_ws/test.sh
```

LeRobot 依赖安装见[转换环境](src/data_collection/README.md#转换环境)，本地 fork 的维护方法见 [third_party/README.md](third_party/README.md)。本仓库脚本包含本机工作空间路径，换机器时需核对路径与依赖。

## 硬件与运行模式

| 设备 | 名称 / 通道 | 相机序列号 |
| --- | --- | --- |
| 第三视角相机 | `cam_h` | `262522071923` |
| 左臂相机 | `cam_l` | `262522071867` |
| 右臂相机 | `cam_r` | `262522070143` |
| 左主臂 / 左从臂 | `can1` / `can3` | — |
| 右主臂 / 右从臂 | `can2` / `can0` | — |

相机按序列号绑定名称，默认 640×480、15 FPS，发布彩色图、深度图及对齐深度图。CAN 接口同时校验 USB 物理路径。四路主从控制、主臂对齐、跟随速度及录制方法见[四路 CAN 主从控制](docs/teleop.md)。

| 运行模式 | 通电设备与控制方向 | 入口与复位 |
| --- | --- | --- |
| 数据采集（已实现） | 四臂通电；主臂 → ROS → 从臂，100 Hz | `teleop.sh --execute --cameras=all`＋`record.sh`；遥操作终端输入 `r` 复位四臂 |
| 一般模型部署（已实现） | 仅双从臂需要通电；模型 → 从臂，主臂可以断电 | `start_hardware.sh mode:=deploy`＋`deploy.sh --execute`；模型退出后 `reset.sh` 只复位从臂 |
| RL 人在环（待实现） | 四臂通电；模型 → 从臂 → 主臂，人工接管时切换为主臂 → 从臂 | 后续实现模型/人工控制权切换、主臂同步与四臂复位；当前没有该模式入口 |

三种模式使用四路独立接线和正常 ACK CAN；硬件管理器内部的 `record/deploy` 是话题与监控配置，并非额外的操作模式。在线只推理用普通部署硬件＋`deploy.sh --dry-run`。数据转换、训练、离线推理和轨迹预览不需要连接机械臂。

切换模式前先停止录制和当前控制程序，再停止硬件管理器。遥操作与独立部署、复位、回放进程共享控制锁；采集的 `r` 在现有节点内执行复位。四路独立接线不应恢复主从 CAN 直连。完整启动选项见[硬件说明](docs/hardware.md)。

## 选择双臂或单臂

以下默认流程仍为双臂。`teleop.sh`、`record.sh`、数据转换、`deploy.sh` 和 `reset.sh` 新增统一参数
`--arms both|left|right`，不传时默认双臂。单臂指一侧 6 个关节＋夹爪，共 **7 维**；
默认仍使用三路相机。选择不会自动切换相机，也不会将双臂模型裁成单臂模型。

**只用左臂采集**：仅左主臂和左从臂需要通电，其他接线保持原状。

```bash
cd ~/AgileX_ws
./prepare_can.sh --mode=teleop can1 can3
./teleop.sh --arms left --configure-only
./teleop.sh --arms left --execute --cameras all
# 回车使能保持，输入 d 对齐并跟随；同一终端 r 只复位左侧主从臂。

# 另一终端录制；每段换一个目录，Ctrl+C 保存。
./record.sh --arms left --output ~/AgileX_ws/bags/task_single_left_0001
```

录制会筛掉另一侧机械臂话题，清单记录所选侧；单臂采集不需要 `--allow-partial`。
`teleop.sh` 与 `record.sh` 的选择要一致。若硬件管理器仍按双臂启动，它仍会等待另一侧，
因此切换前需退出旧的采集/硬件进程，再按新参数启动。

将这些单臂 bag 转为 7 维数据并训练（先采集足够多的 episode）：

```bash
./src/data_collection/scripts/convert_lerobot.sh --arms left \
  --episodes task_single_left_0001 task_single_left_0002 \
  --output ~/AgileX_ws/datasets/piper_single_left
# --episodes 后可列出同批全部目录；省略会扫描转换 YAML 中所有匹配的 bag。
./training/train_act.sh --config_path=training/act_left.yaml
```

转换同时筛选状态、动作、电流/力矩/电机反馈；动作列顺序为所选侧 J1..J6、夹爪宽度。
模板 `training/act_left.yaml` 和 `training/act_right.yaml` 均使用三相机、7 维状态/动作；
默认输出目录已与双臂分开，重复训练需指定新的输出目录。

**只部署左从臂**：停止遥操作及旧硬件进程；仅左从臂和三相机需要通电。

```bash
./prepare_can.sh --mode=deploy can3
./start_hardware.sh mode:=deploy arms:=left

# 另一终端：替换为同侧训练所得的真实 7 维检查点。
./deploy.sh --arms left --checkpoint /path/to/left/pretrained_model --dry-run
./deploy.sh --arms left --checkpoint /path/to/left/pretrained_model --execute
# 模型程序退出后，只复位左从臂；默认仍先开爪、抬升，再回原固定姿态。
./reset.sh --arms left
```

右侧操作将 `left` 换为 `right`，CAN 采集准备为 `can2 can0`、部署准备为 `can0`，训练配置为
`training/act_right.yaml`，数据/模型目录也相应改为右侧。部署检查点必须与采集侧一致；
程序检查 7/14 维契约，但无法仅凭一个没有侧别元数据的 7 维权重判断它训练于左侧还是右侧。
已有双臂 14 维 ACT 检查点不能直接用于 `--arms left/right`，会在控制器使能前拒绝加载。
当前 `replay.sh` 的实机轨迹回放仍按原双臂流程使用。

## 数据采集

### 启动与检查

关闭占用相机的 RealSense Viewer 或其他驱动。终端 1 准备 CAN 并启动硬件：

```bash
~/AgileX_ws/prepare_can.sh --mode=teleop
~/AgileX_ws/teleop.sh --configure-only
~/AgileX_ws/teleop.sh --execute --cameras=all
# 按回车使能保持，再输入 d 对齐主臂并开启跟随。
```

`prepare_can.sh` 需要本机 sudo 权限，准备全部四路 1 Mbps 正常 CAN，配置正确时跳过。独立单臂需要电脑提供 ACK。等待主从跟随开启及 `READY` 后再录制；先停止录制，再暂停或退出遥操作。目标缓存保留原时间戳，源健康另行检查。

在另一终端查看图像：

```bash
source ~/AgileX_ws/env.sh
rosrun rqt_image_view rqt_image_view /cam_h/color/image_raw
```

通过下拉框切换相机或深度图。常用消息与帧率检查见[查看图像与消息](docs/hardware.md#查看图像与消息)。

### 录制与停止

终端 2，每次演示使用一个目录，统一采用 `insert_plug_v2/task_0001` 格式：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/insert_plug_v2/task_0001
```

看到 `Recording ...` 后开始演示。结束时在录制终端按 **Ctrl+C**，等待 `complete` 和命令退出。下一段使用 `task_0002`，依次递增。

```text
bags/insert_plug_v2/task_0001/
├── data.bag          # 原始 ROS 数据，默认 LZ4 无损压缩
├── manifest.json     # 设备、配置、标定和录制检查结果
└── rosbag.log        # 底层录制日志
```

**同一目录再次录制会在硬件与磁盘检查通过后覆盖其中的 bag、清单和日志。** 要保留上一段，请换目录；存在 `data.bag.active` 时拒绝覆盖。默认要求双臂和三相机全部就绪，并保留至少 5 GiB 磁盘空间。

`complete` 表示录制文件通过完整性检查，不保证没有丢帧或时序误差。部分设备采集、失败处理及完整判定见[录制说明](src/data_collection/README.md#录制-rosbag)。

### 采集时复位

在 `teleop>` 终端输入 `r` 后按回车。程序先结束并保存正在录制的当前段，然后暂停跟随、保持主臂，让左右从臂依次执行原复位流程，再让主臂对齐各自从臂。固定关节姿态仍为左右 `[0, 0, 0, 0, 24, 0]°`，以 `act.yaml` 的 `reset.home_joints_rad` 为准。开爪会释放物体，请先解除插接接触并留出路径、放开主臂。

完成后四臂保持，不自动恢复跟随。输入 `d` 重新进入采集，夹爪接管且 `READY` 后再录制下一段。当前段保存失败或任何复位阶段失败都会阻止后续运动；不会把复位动作写入刚结束的演示。不要在遥操作运行时另开 `reset.sh`。

先停止录制，再在硬件终端按 **Ctrl+C**。停止 ROS 发布不等同于机械臂断电或急停。离线检查和 `rosbag play` 见[bag 消息回放](src/data_collection/README.md#查看与回放-bag-消息)；控制实机重复动作见下文[实机轨迹回放](#实机轨迹回放)。

## 数据转换与 ACT 训练

### 转换 LeRobot 数据集

转换无需启动硬件。默认转换 YAML 仍匹配旧的 `bags/task_*`；先按[批次配置说明](src/data_collection/README.md#为采集批次设置输入与输出)，把输入改为 `bags/insert_plug_v2/task_*/data.bag`，并设置输出 `datasets/piper_insert_plug_v2`、标识 `local/piper_insert_plug_v2`。

```bash
# 先扫描话题、字段和时间同步
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh --dry-run
# 检查通过后生成数据集，输出目录必须不存在
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh
```

默认导出 15 FPS、三路 RGB、14 维从臂状态和主臂目标回显动作，并额外保存电流、估算力矩等反馈；深度默认关闭。成功后生成 LeRobot 数据及 `meta/conversion_report.json`。字段顺序、同步与溯源规则见[采集包文档](src/data_collection/README.md)。

### 训练与选择检查点

训练不需要 ROS 或机械臂。完成上述 v2 转换后，以基线配置启动一次新的训练，显式指定该数据集及新的输出目录：

```bash
~/AgileX_ws/training/train_act.sh \
  --config_path="$HOME/AgileX_ws/training/act.yaml" \
  --dataset.repo_id=local/piper_insert_plug_v2 \
  --dataset.root="$HOME/AgileX_ws/datasets/piper_insert_plug_v2" \
  --output_dir="$HOME/AgileX_ws/outputs/act_insert_plug_v2_new_run"
```

运行前确认原任务已退出、输出目录未被使用，并核对配置中的划分比例、batch 和步数。脚本要求 CUDA 可用，训练结束输出 `End of training`，检查点位于输出目录的 `checkpoints/`。

训练输入为三路 RGB＋14 维从臂状态；数据集中的深度、电流和估算力矩不自动参与模型输入。恢复训练、已有 v3 实验及日志见[训练文档](training/README.md)。部署前在[部署配置](src/model_deploy/config/act.yaml)中显式选择 `checkpoint`，同时保留模型及归一化处理器；`last` 仅表示最新检查点。

## 模型部署与实机操作

### 只推理检查与正式部署

普通部署仅需双从臂和相机通电，主臂可断电。先退出遥操作及旧硬件发布，在终端 1 启动部署反馈：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy
```

`--mode=deploy` 默认只准备左从 can3、右从 can0，不检查两台主臂 CAN。终端 2 先检查实时推理：

```bash
~/AgileX_ws/deploy.sh --dry-run --duration=10
```

该命令不创建 SDK 控制器、不发送使能或动作。确认推理结果并准备好从臂、物体初始状态后，在终端 2 执行：

```bash
~/AgileX_ws/deploy.sh --probe-firmware
~/AgileX_ws/deploy.sh --execute
```

固件查询会发送 CAN 请求，但不使能或移动。`--execute` 检查通过后等待回车，默认最多运行 60 秒。**Ctrl+C 请求保持实测姿态及夹爪开口**，不回零、不松爪；反馈失效时改为请求阻尼停止。实际停止效果需要实机验证。

每次运行结果保存在 `logs/deploy/<时间>/`。配置、输出话题、停止限制及恢复采集见[部署文档](src/model_deploy/README.md)。

### 固定姿态复位

一般模型退出后，保持部署模式硬件发布运行；以下只复位两台从臂，不连接或移动主臂：

```bash
~/AgileX_ws/reset.sh --dry-run
~/AgileX_ws/reset.sh
```

默认顺序：开爪至 100 mm → 沿关节路径抬高法兰 10 cm → 回到固定姿态 → 到位后闭合。**开爪会释放夹持物，路径需要预留空间。** `--dry-run` 只打印要求，不检查实机可达性；正式规划失败不会继续运动。Ctrl+C 使用部署程序的停止处理。

固定姿态由 `act.yaml` 的 `reset.home_joints_rad` 设置，不随每次部署改变。抬升、路径限制及日志见[复位说明](src/model_deploy/README.md#复位到固定姿态)。

### 实机轨迹回放

读取 bag 中左右从臂的实测关节和夹爪轨迹，通过 SDK 控制双臂。先离线预览整段轨迹与起点：

```bash
~/AgileX_ws/replay.sh ~/AgileX_ws/bags/insert_plug_v2/task_0001 --dry-run
```

让主臂退出控制，停止旧硬件、其他控制程序及 `rosbag play`，手动将双从臂放到预览起点并准备物体。终端 1：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy cameras:=none
```

终端 2：

```bash
~/AgileX_ws/replay.sh ~/AgileX_ws/bags/insert_plug_v2/task_0001 --execute
```

默认半速，检查通过后按回车开始。完成或 Ctrl+C 时请求保持实测姿态和夹爪开口，不自动复位。日志位于 `logs/replay/<时间>/`。回放不具备避障或环境感知能力，跟随等待可能延长总耗时。

自动移动到起点的 `--move-to-start`、倍率和反馈检查见[完整回放说明](src/model_deploy/README.md#从-bag-回放实机轨迹)。

## 配置与详细文档

RLT 实验代码位于独立的 [LeRobot `test/pi05_rlt` 分支](https://github.com/Jaymeeeee/lerobot/tree/test/pi05_rlt)。
当前决定先按现有实现训练，已知差异暂不修改；清单见[论文核对](docs/rlt_paper_audit.md)，
分支对应的固定提交与换机器克隆方法见[依赖仓库说明](third_party/README.md)。

| 模块 | 配置入口 | 详细说明 |
| --- | --- | --- |
| `hardware_bringup`：设备、相机、健康检查 | [hardware.yaml](src/hardware_bringup/config/hardware.yaml) | [硬件启动与运行模式](docs/hardware.md)、[话题与数据语义](docs/topics.md) |
| `data_collection`：录制与数据转换 | [recording.yaml](src/data_collection/config/recording.yaml)、[lerobot_conversion.yaml](src/data_collection/config/lerobot_conversion.yaml) | [采集包 README](src/data_collection/README.md) |
| `training`：ACT 训练与恢复 | [训练 act.yaml](training/act.yaml) | [训练 README](training/README.md) |
| `model_deploy`：模型控制、复位、轨迹回放 | [部署 act.yaml](src/model_deploy/config/act.yaml)、[replay.yaml](src/model_deploy/config/replay.yaml) | [部署包 README](src/model_deploy/README.md) |
| 本地 LeRobot fork | `third_party/lerobot/` | [仓库维护说明](third_party/README.md) |

`bags/` 保存录制数据，`datasets/` 保存转换数据，`outputs/` 保存训练结果，`logs/` 保存运行与测试日志。具体路径以对应 YAML 或命令行参数为准。

双臂与三相机已完成短时录制验证；历史测试、固件查询和部署验证范围统一见[验证记录](docs/validation.md)。短时验证不代表长期稳定性、实机停止效果或插接成功率已验收。
