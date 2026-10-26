# Piper 操作指南

机械臂控制使用 pyAgxArm SDK，相机使用 RealSense SDK，模型使用 LeRobot。**ROS 只负责数据发布与录包，模型部署和人工接管直接通过 SDK 执行。**

下面以右侧主从臂采集、插头任务 v4 的 ACT 模型为例，按上机顺序操作。

## 1. 通电与准备

连接所需机械臂、USB-CAN 和相机并通电。采集需要右主臂、右从臂；单臂部署只需右从臂。配置未选中的左臂不会连接。

每个控制或录制终端先进入仓库并激活环境：

```bash
cd ~/AgileX_ws
conda activate piper-control
```

本机 `piper-control` 环境运行控制程序，`lerobot` 环境运行模型，系统 Python 运行 ROS Noetic。模型和 ROS 子进程由配置指定解释器，无需在控制终端切换环境。

CAN 与相机按 USB 序列号绑定。CAN 映射集中在 [configs/hardware/can.yaml](configs/hardware/can.yaml)，相机映射在 [configs/base.yaml](configs/base.yaml)。查看当前 CAN 设备：

```bash
python scripts/prepare_can.py --list
```

后续各流程的 `prepare_can.py --execute` 会设置所选接口的波特率与发送队列；USB 重插后重新执行即可。准备 CAN、恢复电子停止前先退出已有控制程序。使能和主从对齐前放稳机械臂并松手；`recover.py` 的复位会暂时失能。

## 2. 数据采集：先启动主从，再手动录包

### 终端 1：启动 ROS

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
/usr/bin/python3 /opt/ros/noetic/bin/roscore
```

已有 roscore 时复用，不重复启动。

### 终端 2：启动右侧主从控制

```bash
export ROS_MASTER_URI=http://localhost:11311
sudo -v
python scripts/prepare_can.py --config configs/collect_right.yaml --execute
python scripts/run.py --config configs/collect_right.yaml --execute
```

程序自动恢复电子停止、使能、将主臂对齐到从臂当前姿态，运动稳定后进入 `HUMAN`，无需输入 `h`。轻缓移动主臂，确认从臂跟随且终端出现“人工 action 数据已就绪”。**此时只发布 ROS 消息，还没有保存 bag。**

### 终端 3：手动录制一段示教

```bash
export ROS_MASTER_URI=http://localhost:11311
python scripts/record.py --config configs/collect_right.yaml --list-topics
python scripts/record.py --config configs/collect_right.yaml --output bags/right_demo/data_0001
```

录制启动后开始示教，完成后在**录制终端按 Ctrl+C** 保存。控制终端继续运行；下一段换成 `bags/right_demo/data_0002` 再录制。同一任务放在同一父目录下，每段使用新目录，已有文件不会覆盖。

每段目录包含 `data.bag`、`manifest.json` 和 `rosbag.log`。录制内容由 [configs/recording/topics.yaml](configs/recording/topics.yaml) 调整，右侧任务包含：

- 顶部 `cam_h`、右腕 `cam_r` 的 RGB、深度、对齐到 RGB 的深度及各自 `camera_info`。
- 右从臂实测状态和诊断、主臂控制数据、合并状态。
- 实际转发的人工动作 `action`、来源信息 `action_metadata`，以及可用的事件和主臂反馈。

主臂进入 leader 模式后的 `leader/command` 是控制目标，不能当作实测状态。结束时会检查必需 topic：缺项则 `manifest.json` 标记 `incomplete` 并列出 `missing_required_topics`；`complete` 表示必需 topic 均收到消息，不代表任务成功或每帧都无丢失。

## 3. 模型部署

### 已训练模型：右从臂 ACT 050000

[configs/act_insert_plug_v4_right.yaml](configs/act_insert_plug_v4_right.yaml) 已指向：

```text
outputs/models/act_piper_insert_plug_v4_20261025/checkpoints/050000/pretrained_model
```

该模型使用右臂 7 维状态／动作、顶部与右腕 RGB，控制频率配置为 15 Hz。仅连接右从臂，不需要 ROS 或主臂。

```bash
sudo -v
python scripts/prepare_can.py --config configs/act_insert_plug_v4_right.yaml --execute
# 上次退出留下电子停止时，先恢复。
python scripts/recover.py --config configs/act_insert_plug_v4_right.yaml --execute
python scripts/run.py --config configs/act_insert_plug_v4_right.yaml --execute --auto-start --seconds 30
```

摆好示教时的起始姿态和物体后再启动。上述命令运行 30 秒后退出；不加 `--auto-start` 时先保持姿态，输入 `p` 再开始。去掉 `--seconds 30` 后，仍受默认 120 秒回合上限约束，到时暂停；该上限由 `workflow.max_episode_sec` 调整。

更换模型时配置 `policy.type` 和 `policy.checkpoint`，并让机械臂侧别、相机输入、状态／动作顺序和单位与训练一致。单臂 7 维、双臂 14 维，不能直接截取双臂模型输出用于单臂。

### 需要训练新模型时

流程为 **bag → LeRobot 数据集 → 训练 → 将部署配置指向新 checkpoint**。转换使用 [configs/lerobot_insert_plug_v4.yaml](configs/lerobot_insert_plug_v4.yaml)，训练使用 [configs/training/act_insert_plug_v4.yaml](configs/training/act_insert_plug_v4.yaml)。

转换前将 `input_dir` 指向本次任务目录（上例为 `bags/right_demo`，可用命令行 `--input bags/right_demo` 指定），并填写任务描述。原 v4 数据集和模型目录已经存在，需指定新的输出目录；训练配置的 `dataset.root` 也要指向新数据集。完整命令见 [数据与训练](docs/learning.md)。

## 4. 人在环：人工接管与强化学习

### 给右臂 ACT 加上人工接管

新建 `configs/act_insert_plug_v4_right_hil.yaml`，继承上面的模型，只增加右主臂：

```yaml
extends: act_insert_plug_v4_right.yaml
robot:
  pairs:
    right:
      leader: true
```

```bash
sudo -v
python scripts/prepare_can.py --config configs/act_insert_plug_v4_right_hil.yaml --execute
python scripts/recover.py --config configs/act_insert_plug_v4_right_hil.yaml --execute
python scripts/run.py --config configs/act_insert_plug_v4_right_hil.yaml --execute
```

启动会使能主从臂并发送同一初始目标，然后保持姿态。输入命令后按回车：`p` 开始模型 → `h` 人工接管 → `p` 恢复模型。接管时清空尚未执行的模型动作，主臂 CAN 命令转发到从臂；恢复时从当前状态和图像重新推理。

**ACT 人工接管不会更新模型权重。** 上述配置也不录制；需要保留纠正轨迹时，设置 `record.backend: ros` 并启动 roscore，会自动保存会话和 bag。录制配置见 [配置说明](docs/configuration.md#工作流与采集彼此独立)。

### π0.5/RLT 在线强化学习

在线更新入口是 [configs/rlt_online.yaml](configs/rlt_online.yaml)，当前为**双臂模板**，checkpoint 仍需填写。需准备自定义 LeRobot 分支、兼容的 `pi05_rlt` checkpoint 和冻结 π0.5 资源，并匹配机械臂及相机配置后运行：

```bash
sudo -v
python scripts/prepare_can.py --config configs/rlt_online.yaml --execute
python scripts/recover.py --config configs/rlt_online.yaml --execute
python scripts/run.py --config configs/rlt_online.yaml --execute
```

一轮流程是：`p` 模型执行 → 必要时 `h` 接管、`p` 恢复 → `s` 成功或 `f` 失败结束回合 → 机械臂保持、构建回放并更新权重 → 更新完成后 `p` 开始下一轮。

RLT 默认以文件保存执行轨迹和训练快照，不需要 ROS；需要 bag 时选择 `record.backend: ros`。在线 learner 当前只支持 `pi05_rlt`。模型准备、奖励、回放与断点恢复见 [RLT 在线学习](docs/learning.md#π05rlt-在线学习)。

## 5. 暂停、退出与失能

以下命令在**控制终端**输入并按回车；录包的启停仍由独立录制终端控制。

| 命令 | 操作 |
| --- | --- |
| `p` | 开始／恢复模型，需要已加载模型 |
| `h` | 人工接管，需要已连接主臂 |
| `hold` | 暂停并保持姿态 |
| `s` / `f` | 成功／失败，结束当前回合 |
| `t` | 截断当前回合 |
| `q` | 正常退出 |
| `stop` / Ctrl+C | 电子停止并退出 |

采集结束先 Ctrl+C 停止录包，再到控制终端输入 `q`。**程序退出会电子停止并断连，但不会自动失能。** 需要释放电机时，先放稳或托住机械臂及夹持物，再单独执行：

```bash
# 右侧主从臂都失能。
python scripts/disable.py --config configs/collect_right.yaml --execute
# 若只需失能右从臂，改用 configs/act_insert_plug_v4_right.yaml。
```

失能范围由所用配置决定，失能后不再主动保持姿态或夹持力。

## 进一步查看

| 内容 | 文档／入口 |
| --- | --- |
| 单／双臂、模型、相机与录制配置 | [配置说明](docs/configuration.md) |
| SDK 调用、CAN、对齐、恢复与轨迹回放 | [硬件说明](docs/hardware.md) |
| bag 转换、训练与 RLT 续训 | [数据与学习](docs/learning.md) |
| 模型、校验报告、运行记录的位置与来源 | [outputs 目录说明](docs/outputs.md) |
| 核心实现 | [工作流](piper_control/workflows/session.py)、[机械臂](piper_control/hardware/robot.py)、[模型适配](piper_control/policies/adapter.py) |
| 离线检查 | `python scripts/check_config.py --config 配置文件`；`python scripts/test.py` |
| 旧工程位置与依赖来源 | [迁移记录](docs/migration.md)、[LeRobot 来源](third_party/README.md) |
