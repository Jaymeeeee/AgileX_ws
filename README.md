# Piper SDK 控制与人在环学习

本项目提供 Piper 单臂／双臂控制、示教数据采集、LeRobot 模型部署和 π0.5/RLT 人在环学习。机械臂与相机分别通过 pyAgxArm、RealSense SDK 访问；ROS 用于发布和录制数据，部署和控制切换直接通过 SDK 执行。工程使用独立 Python 脚本，无需构建 ROS package。

## 1. 环境与设备准备

以下命令均在仓库根目录执行。环境名称仅为示例，`/path/to/...` 需替换为实际路径。

### 运行环境

| 环境 | 用途 | 依赖与配置 |
| --- | --- | --- |
| 控制环境 | 机械臂、相机、工作流与录制入口 | `requirements-control.txt` |
| 模型环境 | 推理与训练 | 对应 LeRobot 版本及模型依赖；通过 `policy.python` 指定 |
| ROS 环境 | 发布标准消息和保存 bag | ROS Noetic；通过 `record.python` 和 `record.ros_setup` 指定 |

创建控制环境：

```bash
conda create -n piper-control python=3.10
conda activate piper-control
python -m pip install -r requirements-control.txt
```

控制依赖不包含 LeRobot 或 ROS。模型运行在独立子进程中；RLT 使用的 LeRobot 分支要求 Python 3.12 及以上，安装来源见 [LeRobot 依赖说明](third_party/README.md)。模型权重、tokenizer 和基础模型资源需提前下载到本地，仓库不附带训练数据和权重。

### 硬件绑定

连接机械臂、USB-CAN 与相机并通电，按实际设备修改以下配置。仓库中的序列号和解释器路径是示例值。

| 配置 | 需要填写的内容 |
| --- | --- |
| [configs/hardware/can.yaml](configs/hardware/can.yaml) | `left_leader`、`left_follower`、`right_leader`、`right_follower` 对应的 USB-CAN 序列号 |
| [configs/base.yaml](configs/base.yaml) | SDK 固件协议、相机序列号、模型解释器及 ROS 环境路径 |
| 具体任务配置 | 所选机械臂、相机映射、模型类型、checkpoint 和控制频率 |

```bash
python scripts/prepare_can.py --list
```

CAN 接口通过 USB 标识解析，接口编号变化后重新执行相应任务的 CAN 准备命令即可。准备接口前应退出已有控制程序；使能和主从对齐前需放稳机械臂并松手。`recover.py` 用于解除电子停止，调用 SDK 复位时会暂时失能。

## 2. 示教数据采集

采集流程为：**启动主从控制与 ROS 发布 → 手动开始录包 → 示教 → 停止录包**。根据设备选择配置：

| 场景 | 配置 | 连接设备与相机 |
| --- | --- | --- |
| 单臂示教（右侧） | [collect_right.yaml](configs/collect_right.yaml) | 右主臂、右从臂；顶部和右腕相机 |
| 双臂示教 | [collect_dual.yaml](configs/collect_dual.yaml) | 左右主从臂；顶部、左腕和右腕相机 |

两个配置均自动进入主从控制，并使用 `ros_publish` 后端；启动控制程序时尚不创建 bag。已有 [collect.yaml](configs/collect.yaml) 则保留自动录包模式，行为见 [配置说明](docs/configuration.md)。

### 启动 ROS

在独立终端运行；已有 roscore 时可复用：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
/usr/bin/python3 /opt/ros/noetic/bin/roscore
```

### 启动主从控制

在控制环境终端选择配置。双臂示教将下方变量改为 `configs/collect_dual.yaml`：

```bash
PIPER_CONFIG=configs/collect_right.yaml
export ROS_MASTER_URI=http://localhost:11311
sudo -v
python scripts/prepare_can.py --config "$PIPER_CONFIG" --execute
python scripts/run.py --config "$PIPER_CONFIG" --execute
```

程序按需恢复电子停止、使能并将主臂移向从臂当前姿态，运动稳定后自动进入 `HUMAN`。确认主从跟随正常，且终端出现“人工 action 数据已就绪”后开始录制。

### 录制与检查

另开控制环境终端，`PIPER_CONFIG` 必须与主从控制终端一致：

```bash
PIPER_CONFIG=configs/collect_right.yaml
export ROS_MASTER_URI=http://localhost:11311
python scripts/record.py --config "$PIPER_CONFIG" --list-topics
python scripts/record.py --config "$PIPER_CONFIG" --output bags/task_name/episode_0001
```

录制启动后进行示教，结束时在**录制终端按 Ctrl+C**。下一段使用新的目录，例如 `episode_0002`；主从控制可持续运行。

每段保存 `data.bag`、`manifest.json` 和 `rosbag.log`。录制清单在 [configs/recording/topics.yaml](configs/recording/topics.yaml)，随所选机械臂和相机展开，包含从臂状态、主臂命令、实际转发动作及来源，以及相机 RGB、深度、对齐深度和标定信息。

主臂的 `leader/command` 表示控制目标，从臂 `joint_states` 表示实测状态。缺少必需 topic 时，manifest 标记 `incomplete` 并列出缺项；`complete` 只表示必需 topic 均收到消息，不表示任务成功或逐帧无丢失。

## 3. 单臂与双臂模型部署

部署前编辑所选配置的 `policy` 字段，填写模型类型、checkpoint 和解释器。以下以 ACT 为例：

```yaml
policy:
  backend: lerobot
  type: act
  checkpoint: /path/to/pretrained_model
  python: /path/to/lerobot/bin/python
  device: cuda
```

checkpoint 必须包含模型配置、权重及保存的预处理／后处理配置。机械臂侧别、相机名称、状态／动作排列和单位必须与训练一致；控制频率通过 `workflow.policy_hz` 设置。

| 场景 | 配置 | 状态／动作维度 | 默认相机 |
| --- | --- | --- | --- |
| 左单臂 | [act_left.yaml](configs/act_left.yaml) | 7 | 顶部、左腕 |
| 右单臂 | [act_right.yaml](configs/act_right.yaml) | 7 | 顶部、右腕 |
| 双臂 | [act_dual.yaml](configs/act_dual.yaml) | 14 | 顶部、左腕、右腕 |

每臂包含六个关节角（rad）与夹爪宽度（m）；双臂默认按左、右排列。双臂 checkpoint 不能通过截取输出直接用于单臂。上述部署配置只连接从臂，`record.backend: none`，无需 ROS。

### 单臂部署示例

以右从臂为例；使用左臂时将配置替换为 `configs/act_left.yaml`：

```bash
sudo -v
python scripts/prepare_can.py --config configs/act_right.yaml --execute
# 上次退出后仍处于电子停止时，先恢复。
python scripts/recover.py --config configs/act_right.yaml --execute
python scripts/run.py --config configs/act_right.yaml --execute
```

### 双臂部署示例

```bash
sudo -v
python scripts/prepare_can.py --config configs/act_dual.yaml --execute
# 上次退出后仍处于电子停止时，先恢复。
python scripts/recover.py --config configs/act_dual.yaml --execute
python scripts/run.py --config configs/act_dual.yaml --execute
```

启动会加载模型和相机、使能所选从臂，然后保持姿态。输入 `p` 并回车开始推理，`hold` 暂停，`q` 退出。已准备好任务场景时，可追加 `--auto-start` 自动开始；`--seconds 30` 将启动完成后的整个会话限制为 30 秒。默认回合上限为 120 秒，到时进入保持状态，可通过 `workflow.max_episode_sec` 调整。

训练流程为 **示教数据 → LeRobot 数据集 → 训练 → 配置新 checkpoint**。通用训练模板为 [configs/training/act.yaml](configs/training/act.yaml)。现有 bag 转换器面向右臂、顶部与右腕相机；双臂数据需补充对应转换实现，不能仅更换部署配置完成转换。详见 [数据转换与训练](docs/learning.md)。

## 4. RLT 人在环强化学习

以 π0.5/RLT 为例，模型负责自主执行，操作者通过主臂接管；回合结束后，系统利用实际执行轨迹构建回放并更新策略。普通 ACT 部署中的人工接管仅切换控制权，在线 learner 当前支持自定义 `pi05_rlt`。

### 配置模型与主从臂

使用 [configs/rlt_online.yaml](configs/rlt_online.yaml)。该配置默认选择双臂，并启用左右主臂和从臂，关键字段为：

```yaml
policy:
  type: pi05_rlt
  checkpoint: /path/to/pi05_rlt/pretrained_model
  python: /path/to/lerobot/bin/python
  lerobot_source: /path/to/lerobot
workflow:
  mode: online_rl
  learner: pi05_rlt
record:
  backend: files
```

模型环境需安装 [指定的 LeRobot 分支](third_party/README.md)，准备兼容的 RLT 推理 checkpoint、冻结 π0.5 和 tokenizer 资源。资源路径与 checkpoint 保存值不同时，通过 `policy.overrides` 指定，详见 [RLT 模型与学习配置](docs/learning.md#π05rlt-在线学习)。

单臂 RLT 需要对应单臂 checkpoint，同时修改 `robot.arms` 和 `observation.camera_map`。每个所选侧都必须启用主臂与从臂；模型观测和动作仍按从臂组织。

### 运行与人工介入

```bash
sudo -v
python scripts/prepare_can.py --config configs/rlt_online.yaml --execute
# 上次退出后仍处于电子停止时，先恢复。
python scripts/recover.py --config configs/rlt_online.yaml --execute
python scripts/run.py --config configs/rlt_online.yaml --execute
```

启动时使能主从臂、发送同一初始目标，然后保持姿态。每轮按以下流程操作：

1. `p`：模型根据当前状态与图像执行动作。
2. `h`：人工接管，清空尚未执行的模型动作，由主臂控制从臂；再次输入 `p` 恢复模型。
3. `s` / `f`：标记成功／失败并结束回合；`t` 表示截断回合。
4. 系统进入保持状态，构建回放、更新权重并保存快照；更新结束后输入 `p` 开始下一轮。

默认 `files` 后端将轨迹和训练快照保存到 `outputs/sessions/`。需要同时保存 ROS bag 时，将后端改为 `ros` 并启动 roscore；在线学习不能使用只发布、不保存轨迹的 `ros_publish`。奖励、回放、离线更新与断点恢复见 [学习说明](docs/learning.md)。

## 5. 暂停、退出与失能

以下命令在控制终端输入并按回车；独立录包的启停由录制终端控制。

| 命令 | 操作 |
| --- | --- |
| `p` | 开始／恢复模型，需要已加载模型 |
| `h` | 人工接管，需要已连接主臂 |
| `hold` | 暂停并保持姿态 |
| `s` / `f` | 标记成功／失败，结束当前回合 |
| `t` | 截断当前回合 |
| `q` | 正常退出 |
| `stop` / Ctrl+C | 电子停止并退出 |

采集结束先停止录包，再退出控制程序。**退出会电子停止并断连，不会自动失能。** 需要释放电机时，先放稳或托住机械臂及夹持物，再使用与任务相同的配置执行失能：

```bash
# 示例：双臂示教使用的四台机械臂。
python scripts/disable.py --config configs/collect_dual.yaml --execute
```

设备范围由 `robot.arms` 和主从臂开关决定。失能后机械臂和夹爪不再主动保持姿态或夹持力。

## 参考文档

| 内容 | 入口 |
| --- | --- |
| 配置字段、硬件绑定与录制 topic | [配置说明](docs/configuration.md) |
| SDK 调用、CAN、对齐与恢复 | [硬件说明](docs/hardware.md) |
| 数据转换、模型训练与 RLT 续训 | [数据与学习](docs/learning.md) |
| 输出目录及实验记录 | [outputs 说明](docs/outputs.md) |
| 核心实现 | [工作流](piper_control/workflows/session.py)、[机械臂](piper_control/hardware/robot.py)、[模型适配](piper_control/policies/adapter.py) |
| 离线验证 | `python scripts/check_config.py --config 配置文件`；`python scripts/test.py` |
