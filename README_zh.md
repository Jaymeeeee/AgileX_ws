# AgileX_ws：双组 Piper 主从臂与三台 D435

ROS 1 Noetic 工作空间，用于硬件状态发布、rosbag 采集、LeRobot 数据转换、ACT 训练及双从臂部署。默认采集模式下 CAN 只接收；部署、复位和实机轨迹回放由独立 SDK 控制器下发动作。

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
| 左组主从臂 | `can0` | — |
| 右组主从臂 | `can1` | — |

相机按序列号绑定名称，默认 640×480、15 FPS，发布彩色图、深度图及对齐深度图。USB-CAN 重插后需确认左右映射。

| 场景 | CAN / 硬件模式 | 入口 |
| --- | --- | --- |
| 演示采集、在线只推理检查 | LISTEN-ONLY / 默认 `record` | `record.sh`、`deploy.sh --dry-run` |
| 模型控制、复位、实机轨迹回放 | 正常收发 / `mode:=deploy` | `deploy.sh --execute`、`reset.sh`、`replay.sh --execute` |
| 数据转换、训练、离线推理、轨迹预览 | 无需连接硬件 | 转换脚本、训练脚本、`deploy.sh --offline`、`replay.sh --dry-run` |

切换模式前先停止录制或 SDK 控制程序，再停止硬件管理器。部署模式下两侧主臂须退出控制，静止不代表退出。完整启动选项、就绪状态和恢复采集步骤见[硬件说明](docs/hardware.md)。

## 数据采集

### 启动与检查

关闭占用相机的 RealSense Viewer 或其他驱动。终端 1 准备 CAN 并启动硬件：

```bash
~/AgileX_ws/prepare_can.sh
~/AgileX_ws/start_hardware.sh
```

`prepare_can.sh` 需要本机 sudo 权限，默认准备 1 Mbps、LISTEN-ONLY 接口；正在以其他模式运行时拒绝隐式切换。等待 `READY` 后再录制。采集模式下所选侧的主臂关节和夹爪目标须至少收到一次；尚未收到时为 `DEGRADED`。目标缓存保留原时间戳，不能证明主臂持续在线。

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

保持采集模式硬件运行，在第二终端检查实时推理：

```bash
~/AgileX_ws/deploy.sh --dry-run --duration=10
```

该命令不创建 SDK 控制器、不发送使能或动作。检查推理日志与预测结果后，正式控制前让主臂退出控制，准备好从臂与物体的初始状态，停止旧硬件及其他控制进程。终端 1 切换模式并启动反馈：

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy
```

终端 2：

```bash
~/AgileX_ws/deploy.sh --probe-firmware
~/AgileX_ws/deploy.sh --execute
```

固件查询会发送 CAN 请求，但不使能或移动。`--execute` 检查通过后等待回车，默认最多运行 60 秒。**Ctrl+C 请求保持实测姿态及夹爪开口**，不回零、不松爪；反馈失效时改为请求阻尼停止。实际停止效果需要实机验证。

每次运行结果保存在 `logs/deploy/<时间>/`。配置、输出话题、停止限制及恢复采集见[部署文档](src/model_deploy/README.md)。

### 固定姿态复位

模型退出后，保持部署模式硬件发布运行：

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

| 模块 | 配置入口 | 详细说明 |
| --- | --- | --- |
| `hardware_bringup`：设备、相机、健康检查 | [hardware.yaml](src/hardware_bringup/config/hardware.yaml) | [硬件启动与运行模式](docs/hardware.md)、[话题与数据语义](docs/topics.md) |
| `data_collection`：录制与数据转换 | [recording.yaml](src/data_collection/config/recording.yaml)、[lerobot_conversion.yaml](src/data_collection/config/lerobot_conversion.yaml) | [采集包 README](src/data_collection/README.md) |
| `training`：ACT 训练与恢复 | [训练 act.yaml](training/act.yaml) | [训练 README](training/README.md) |
| `model_deploy`：模型控制、复位、轨迹回放 | [部署 act.yaml](src/model_deploy/config/act.yaml)、[replay.yaml](src/model_deploy/config/replay.yaml) | [部署包 README](src/model_deploy/README.md) |
| 本地 LeRobot fork | `third_party/lerobot/` | [仓库维护说明](third_party/README.md) |

`bags/` 保存录制数据，`datasets/` 保存转换数据，`outputs/` 保存训练结果，`logs/` 保存运行与测试日志。具体路径以对应 YAML 或命令行参数为准。

双臂与三相机已完成短时录制验证；历史测试、固件查询和部署验证范围统一见[验证记录](docs/validation.md)。短时验证不代表长期稳定性、实机停止效果或插接成功率已验收。
