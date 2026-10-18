# data_collection：录制、检查与数据转换

[返回工作空间 README](../../README.md) · [硬件启动](../../docs/hardware.md) · [ACT 训练](../../training/README.md)

硬件录制用 ROS Noetic / 系统 Python；离线转换用独立的 LeRobot Python >= 3.12 环境。本文通用采集目录采用 `bags/insert_plug_v2/task_0001`，每次演示一个目录，后续依次使用 `task_0002`、`task_0003`。

## 录制 rosbag

终端 1 按[硬件启动说明](../../docs/hardware.md#采集模式启动与停止)启动采集模式并等待 `READY`。终端 2 指定采集目录，执行后开始录制：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/insert_plug_v2/task_0001
```

看到 `Recording ...` 后开始演示；任务结束时在终端 2 按 **Ctrl+C**，等待 `complete` 和命令退出，完成保存。目录自动创建，文件统一放在其中：

```text
bags/insert_plug_v2/task_0001/
├── data.bag          # ROS 数据，LZ4 无损压缩，不自动分包
├── manifest.json     # 设备、配置、标定、起止时间、话题消息数与录制结果
└── rosbag.log        # 底层录制日志
```

`--output` 必填，指定目录而不是 `.bag` 文件，支持绝对路径、相对路径和 `~`。含空格的路径用引号包住。

**再次指定同一目录，会在硬件就绪、磁盘检查通过后覆盖其中的 `data.bag`、清单和日志**；目录中的其他文件保留。采集不满意时，停止后重新运行同一条命令即可重录；要保留上一段，则换目录：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/insert_plug_v2/task_0002
~/AgileX_ws/record.sh --output="/home/robot/AgileX_ws/bags/insert_plug_v2/task_0003"
```

存在 `data.bag.active` 时拒绝覆盖。同一 ROS master 同时只允许一个本包录制进程。

默认要求两组机械臂及三台相机全部就绪；未就绪时最多等待 120 秒，超时退出。若硬件只启动了部分设备，追加 `--allow-partial`，所选设备仍须就绪：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/insert_plug_v2/task_0004 --allow-partial
```

默认至少保留 5 GiB 磁盘空间。硬件异常、心跳过期、配置变化、磁盘不足或录制进程退出会结束录制并标为 `incomplete`。`complete` 表示 bag 可读、必需话题有消息、无遗留 `.active` 且未检测到缓冲溢出，不保证无任何丢帧或时序误差。先停止录制，再关闭硬件。

消息保留原始 header，bag 时间为录制端接收时间；静止主臂目标保留最后值及原时间戳。不做插值、重采样、跨设备同步或训练格式转换。

## 查看与回放 bag 消息

录制退出并完成保存后，可离线检查 bag：

```bash
source ~/AgileX_ws/env.sh
rosbag info ~/AgileX_ws/bags/insert_plug_v2/task_0001/data.bag
```

先停止在线硬件，避免同名话题混流，再重新发布记录的话题：

```bash
rosbag play ~/AgileX_ws/bags/insert_plug_v2/task_0001/data.bag
```

按 **Ctrl+C** 停止消息回放。`rosbag play` 只重新发布话题，本工作空间的机械臂状态话题不会直接驱动 SDK。让实机重复动作需使用[实机轨迹回放](../model_deploy/README.md#从-bag-回放实机轨迹)，并切换到部署模式；不要与在线硬件或 SDK 控制程序同时运行消息回放。

## bag → LeRobot

入口为 `scripts/convert_lerobot.sh`，默认加载 [config/lerobot_conversion.yaml](config/lerobot_conversion.yaml)。无需启动 ROS、相机或机械臂；不上传数据，不修改原始 bag。

### 为采集批次设置输入与输出

默认配置仍匹配旧目录 `bags/task_*/data.bag`，输出 `datasets/piper_insert_plug`。转换本文的 v2 录制前，在该 YAML 中将以下三项改为对应批次；也可在同一配置目录复制一份 YAML，再用 `--config` 指定：

```yaml
input:
  bag_glob: ../../../bags/insert_plug_v2/task_*/data.bag
dataset:
  repo_id: local/piper_insert_plug_v2
  root: ../../../datasets/piper_insert_plug_v2
```

这里只展示需要调整的字段，其他配置保留。相对路径按 YAML 所在目录解析，复制到其他目录时须相应调整。只传 `--output` 会覆盖输出目录，不会更改输入匹配规则或数据集标识。

### 检查与转换

```bash
# 先检查全部话题、消息字段和时间同步，不写数据集
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh --dry-run

# 检查通过后，转换 YAML 中匹配到的所有 bag
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh

# 指定配置及新的输出目录
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh \
  --config ~/AgileX_ws/src/data_collection/config/lerobot_conversion.yaml \
  --output ~/AgileX_ws/datasets/piper_insert_plug_v2_new

# 预览：第一条 bag 的前 30 个采样帧（不能当作完整数据集）
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh \
  --episodes task_0001 --max-frames 30 \
  --output ~/AgileX_ws/datasets/preview
```

`--episodes` 接受多个采集目录名；`--limit-episodes N` 仅处理前 N 条。输出目录必须不存在，避免覆盖已生成的数据。转换中先写入同级 `.名称.converting-随机串`，成功验证后才改成正式目录；失败/中断时保留临时目录和失败报告，不把部分结果当作完整数据。

### 已有 v3 实验配置

历史 `insert_plug_v3` 批次使用独立配置 [config/lerobot_insert_plug_v3.yaml](config/lerobot_insert_plug_v3.yaml)，输入为 `bags/insert_plug_v3/data_*/data.bag`，输出为 `datasets/piper_insert_plug_v3`，标识为 `local/piper_insert_plug_v3`。沿用主臂目标作为 action、从臂反馈作为 state，三路 RGB 和电机反馈开启，深度关闭：

```bash
~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh \
  --config ~/AgileX_ws/src/data_collection/config/lerobot_insert_plug_v3.yaml
```

### 转换环境

启动脚本默认使用 `~/miniconda3/envs/lerobot/bin/python`，并隔离 ROS 的 `PYTHONPATH` 和动态库。其他环境可指定：

```bash
LEROBOT_PYTHON=/path/to/env/bin/python \
  ~/AgileX_ws/src/data_collection/scripts/convert_lerobot.sh --dry-run
```

本机已安装 `rosbags`。换机器时，在 LeRobot 环境中执行：

```bash
python -m pip install -r ~/AgileX_ws/src/data_collection/requirements-conversion.txt
python -m pip install -e "$HOME/AgileX_ws/third_party/lerobot[training]"
```

实现参考本地 `third_party/lerobot/examples/port_datasets/port_droid.py`，使用官方 `LeRobotDataset.create/add_frame/save_episode/finalize` API。转换报告记录实际 LeRobot 模块路径、Git commit 和依赖版本；当前实现适配本地 0.6.2 源码，不应直接替换成旧版 API。

## YAML 配置

| 配置 | 含义 |
| --- | --- |
| `input.bag_glob` | 输入匹配规则；每个 bag 是一个独立 episode，按路径排序 |
| `input.require_complete_manifest` | 默认只接受清单标记 `complete` 的录制 |
| `dataset.root/repo_id/fps/task` | 本地输出目录、数据集标识、采样率和任务文本；repo_id 不会触发上传 |
| `tasks` | 按采集目录覆盖任务标签 |
| `sources` | 话题、时间戳来源、采样方式、时间容差、输入关节顺序 |
| `vectors` | 输出数值特征，各段按 YAML 顺序拼接；`names` 明确每维含义 |
| `images` | 输出图像、RGB/深度类型、存储方式和启用开关 |
| `video` | RGB 编码参数，默认 H.264 / CRF 18 |

YAML 中相对路径以该 YAML 的目录为基准；CLI `--output` 的相对路径以当前终端目录为基准。路径支持 `~`。

默认导出：

| 特征 | 内容 |
| --- | --- |
| `observation.state` | 14 维从臂反馈：左 J1…J6、左夹爪、右 J1…J6、右夹爪；rad / m |
| `action` | 同顺序的 14 维绝对位置目标回显；不是独立主臂实测反馈或下一帧从臂状态 |
| `observation.images.cam_h/cam_l/cam_r` | 三路 RGB，原尺寸，默认视频存储 |
| `observation.motor_current_a` | 左六轴＋右六轴电流，A |
| `observation.torque_estimate_nm` | 左六轴＋右六轴电流换算力矩，N·m；不是外部接触力矩 |
| `observation.motor_position_rad` | SDK 电机轴位置，rad |
| `observation.motor_velocity_rad_s` | SDK 电机轴速度，rad/s |

电机四类数据各 12 维，单独保存，默认不拼进 `observation.state`。训练时需明确模型输入；不能以“数据集含此字段”推断模型使用了它。原始状态 JSON、events 和诊断仍保存在 bag，没有伪装成模型输入。

默认关闭六路深度映射。将所需 `observation.depth.*` / `observation.aligned_depth.*` 的 `enabled` 改为 `true` 可启用。深度采用 LeRobot 原生 `image` 特征，带 `is_depth_map` 标记，保留 uint16 毫米值和无效值 0，不压成 8 位视频。RGB 默认是有损视频，若需无损保存也可改为 `storage: image`。启用深度后，训练时应明确只选 RGB，或适配深度输入模型。

## 同步与溯源

- 默认按 `header.stamp` 建立均匀 15 FPS 时间轴。所有连续源有数据的公共区间之外仅裁掉首尾；中间缺失超限时失败，不删除中间帧后缩短时间轴。
- 状态、电流、力矩用采样时刻之前的最新反馈，最大年龄 100 ms。图像取最近帧，绝对时间差上限 50 ms；三相机没有硬件同步，保留各自源时间以便评估。
- 主臂关节/夹爪目标按事件保持：在下一次目标到来前使用上次值，绝不取未来目标。初始 latched 消息允许早于 bag 开始；缺少首个有效目标时不会填零。
- 默认图像允许在相邻采样点复用同一源帧，数量写入报告。`hold_last` 允许目标长时间不更新，不证明主臂在线；原始年龄、重复数、乱序数均可追溯。
- 所有数值转为 float32；不做平滑、动作偏移、关节限幅或夹爪零点修正。保留负夹爪读数等原值，后续训练/部署时单独处理标定。

输出包含 LeRobot 的 `data/`、`videos/`、`meta/`（image 存储还可能包含 `images/`）。v3 格式可以让多个 episode 共享 Parquet/视频分片，episode 边界由元数据记录。

额外追溯文件：

```text
meta/
├── conversion_config.yaml
├── conversion_report.json
└── source/episode_000000/
    ├── recording_manifest.json   # 原始设备配置、相机标定等
    ├── conversion.json           # 对应 bag、帧数、裁边与同步误差
    └── timing.npz                # 绝对时间轴、各源 header/bag 时间及原消息序号，int64 ns
```

标准 `timestamp` 为 episode 内从 0 开始的秒数；绝对源时间在 `timing.npz`。成功前检查所有数值行与源采样 float32 值的 SHA-256 一致、所有帧编号/时间连续，并解码每个 episode 的首/中/末图像。完整录制清单和这些检查不等同于任务成功率或模型训练效果验证。

## 测试

```bash
conda activate lerobot
unset PYTHONPATH
export PYTHONNOUSERSITE=1 LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
python -m unittest discover \
  -s ~/AgileX_ws/src/data_collection/tests/conversion -v
```

测试包含真实 ROS 1 bag 写入/转换/读回、关节重排、目标因果保持、时间缺口拒绝、RGB 行填充，以及深度大小端、0 和 65535 无损往返。ROS 录制原有测试仍由工作空间 `test.sh` 执行。
