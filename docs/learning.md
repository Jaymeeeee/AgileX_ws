# 数据、模型与 RLT

## 会话数据

启用采集时，每次运行新建独立目录，不覆盖已有会话：

```text
<record.output_dir>/<session>/
  manifest.json
  trajectory.jsonl
  images/
  capture.bag                  # 仅 record.backend=ros
  training/episode_000001/      # 在线 RLT 快照
```

轨迹以控制流程提交的目标作为训练 action，模型预测单独保存。直接命令版中，模型 action 是 SDK 调用入参（`action_evidence: sdk_target`），人工 action 是已转发 CAN 的解码目标（`forwarded_can_target`）；不再校验 SDK 内部量化、裁剪或底层发送结果。每步包括前后观测、原始时间戳、持续时间、动作来源 BASE/RL/HUMAN、episode/step、控制代次和片段边界。图像保留为无损 PPM；可选深度为原始 NPY 与比例信息。主臂目标和从臂反馈分开记录。

稀疏终止奖励写在 `episode_end` 一次，步骤 reward 默认为零。正常终止关闭 bootstrap；时间截断保留 bootstrap。最终观测使用结束标注时读取的 SDK 缓存，包括暂停后的状态变化。缺少后继观测时仍保留已提交动作，标记不完整，失败会话不作为完整训练数据导出。

ROS 子进程发布 `/piper/state`、各臂状态和 diagnostics、相机 RGB／标定、实际 action、action metadata 与事件，并将同一消息写入 `capture.bag`。它只接收采集数据，不连接机械臂。队列、磁盘和录包失败传回工作流，停止该采集会话。

## 通用模型训练

### insert_plug_v4 的 ROS bag 转换

手工录包目录 `bags/insert_plug_v4/data_*/data.bag` 使用独立入口，不需要
`trajectory.jsonl`。转换配置在 [`configs/lerobot_insert_plug_v4.yaml`](../configs/lerobot_insert_plug_v4.yaml)，
默认输出 `datasets/piper_insert_plug_v4`，使用当前本地 LeRobot 0.6.2 的 v3.0 数据格式。

```bash
# 检查所有 bag 的字段、动作来源和时间同步，不生成数据集。
bash scripts/convert_lerobot.sh --inspect

# 全量转换；输出目录必须不存在。
bash scripts/convert_lerobot.sh

# 只转换一个 bag 验证环境，需要指定另外的输出目录。
bash scripts/convert_lerobot.sh --limit 1 --output /tmp/insert_plug_v4_preview
```

脚本使用 `$HOME/miniconda3/envs/lerobot/bin/python`；可通过 `LEROBOT_PYTHON` 指定其他
已具备 LeRobot、rosbags、PyAV、NumPy、PyYAML 的解释器。现有 LeRobot 环境已具备这些依赖。
shell 入口隔离 ROS Noetic 的 Python 路径，不启动 ROS、机械臂或相机，不上传数据。
也可在合适的模型环境直接运行 `python scripts/bag_to_lerobot.py --help`。
`--input`、`--output` 可替换目录，`--report` 将检查详情保存到一个新的 JSON 文件；
FPS、任务文本、同步容差和视频编码参数在 YAML 中修改。

| LeRobot 字段 | ROS 数据源 | 含义 |
| --- | --- | --- |
| `observation.state` | `/piper/arms/right/follower/joint_states.position` | 右从臂实际状态 |
| `action` | `/piper/action.position` | 主臂已成功转发给从臂的绝对位置目标 |
| `observation.images.cam_h` | `/piper/cameras/cam_h/color/image_raw` | 顶部 RGB |
| `observation.images.cam_r` | `/piper/cameras/cam_r/color/image_raw` | 右腕 RGB |

原配置的状态与动作均为 7 维，按消息的 `name` 重排成 `right_joint_1` 至 `right_joint_6`、
`right_gripper_width`。前六项为 rad，最后一项为 m；不重复做单位转换、归一化、裁剪或差分。
本次 `/piper/state` 是同一从臂状态的汇总；`/leader/command` 是独立采样的主臂命令流，
不能当作实测反馈，也不能替代已经转发的动作标签。脚本逐条匹配 `/piper/action_metadata`
的源时间戳、关节名称与数值，并要求 `source=HUMAN`、`action_evidence=forwarded_can_target`。
仅裁掉录包首尾无法验证来源的动作；中间来源缺失或数值不符会报错。

每个 bag 单独生成一个 episode，不使用持续遥操过程中一直为 0 的 metadata episode ID 合并文件。
先裁到状态、已验证动作及双相机的共同时间范围，再按 15 Hz 定频采样；每路取当前时刻之前
最近一条数据，禁止未来值补齐，默认最大样本年龄 150 ms。动作原始频率约 11–12 Hz，
因此部分目标会保持到下一采样点。同步使用 `header.stamp`，不使用有接收延迟的 bag time。
中间超时直接失败，不删掉缺帧后压缩时间轴；不推断额外动作时延或提前一帧。

RGB 保持 640×480，输出 H.264 视频（CRF 18，有损编码）。原配置仅导出上表四个训练字段：
深度、相机标定与诊断仍保留在原 bag；需要电机反馈时使用下方的独立配置。
尤其深度已是 `32FC1`、单位 m、无效像素为 NaN，
不能按原始 uint16/mm 解码；若后续需要 RGB 对齐深度，应选 `aligned_depth_to_color`，
并单独设计有效像素掩码与统计处理。空的 velocity/effort 不补零，manifest 的 `complete`
不解释为任务成功，不生成臆测的 success/reward 标签。

转换前先检查全部源文件，转换后使用官方 LeRobot reader 逐值核对所有状态/动作、episode
索引与时间轴，并解码每个 episode 两路视频的首尾帧。成功后才将暂存目录改名为目标目录；
失败会保留带 `status=failed` 报告的隐藏暂存目录，已有输出不会覆盖。
`meta/conversion.json` 记录每个 episode 的来源 bag、裁剪量、来源验证、采样年龄和数值哈希；
`meta/source_timestamps/episode_*.npz` 保存每帧的原始源时间、接收时间、消息序号及采样网格，
便于追溯。LeRobot 标准 `timestamp` 则从 episode 开头按帧编号/FPS 计算。

官方格式和 `finalize()` 要求见 [LeRobotDataset v3 文档](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)。

### 将电流和估算力矩加入 observation.state

[`configs/lerobot_insert_plug_v4_with_motor_state.yaml`](../configs/lerobot_insert_plug_v4_with_motor_state.yaml)
从同一批 bag 生成独立数据集 `datasets/piper_insert_plug_v4_with_motor_state`，
标识为 `local/piper_insert_plug_v4_with_motor_state`。它将右从臂六个电机的电流和估算力矩
追加到 `observation.state`，得到 19 维状态；`action` 仍是主臂已转发的 7 维绝对位置目标，
顶部和右腕 RGB 的字段及编码保持相同。原 `datasets/piper_insert_plug_v4` 不会被修改，
新输出目录也必须不存在。

状态顺序固定如下，表中切片采用 Python 的左闭右开规则：

| `observation.state` 切片 | 特征名称 | 单位与来源 |
| --- | --- | --- |
| `[0:7]` | `right_joint_1` 至 `right_joint_6`，`right_gripper_width` | 从臂 `joint_states.position`；关节 rad、夹爪 m |
| `[7:13]` | `right_joint_1_current` 至 `right_joint_6_current` | 从臂 diagnostics 中各电机 `current`；A |
| `[13:19]` | `right_joint_1_torque` 至 `right_joint_6_torque` | 从臂 diagnostics 中各电机 `torque`；N·m |

电机数据来自 `/piper/arms/right/follower/diagnostics`，不取主臂诊断，也不从空的
`JointState.effort` 补值。这里的 `torque` 是 SDK 按 `current × k × b × c` 换算得到的估计，
不是独立力矩传感器的测量值；转换直接保留 bag 中的反馈值，不重复换算。

diagnostics 是缓存状态的 JSON 快照，外层消息时间不能代表六个电机均在该时刻更新。
脚本分别建立 `motor_1` 至 `motor_6` 的时间序列，使用每个电机反馈条目内嵌的 `stamp`
与原有状态、动作和相机对齐。每路仍只取采样时刻之前的最新值，并检查 150 ms 的默认
最大样本年龄；任一电机缺失或过期会报错，不将旧缓存重新标记为新数据。
电机源时间也写入 `meta/source_timestamps/episode_*.npz`，可逐帧追溯。

```bash
# 检查完整批次的电机字段及每个电机的时间同步。
bash scripts/convert_lerobot.sh \
  --config configs/lerobot_insert_plug_v4_with_motor_state.yaml --inspect

# 输出新的 19 维状态数据集。
bash scripts/convert_lerobot.sh \
  --config configs/lerobot_insert_plug_v4_with_motor_state.yaml

# 仅转换一个 bag 检查环境，使用另外的输出目录。
bash scripts/convert_lerobot.sh \
  --config configs/lerobot_insert_plug_v4_with_motor_state.yaml \
  --limit 1 --output /tmp/insert_plug_v4_with_motor_state_preview
```

训练时应将数据集路径指向新目录，并让模型使用上述 19 维状态及相应的归一化统计。
部署时也必须构造相同顺序、单位和采样语义的 19 维观测；现有只提供 7 维状态的部署输入
不能直接用于该模型。本次数据转换不会自动修改部署流程或已有模型的输入契约。

### 仅增加电流的 ACT 对照实验

`scripts/select_dataset_state.py` 从已有 19 维数据集中保留前 13 项，生成独立目录
`datasets/piper_insert_plug_v4_with_current`：原有 7 维状态，加六个从臂关节电流。
脚本同步裁剪全局、逐回合统计及 Parquet 特征定义，逐字节复制视频，并用官方 reader
验证全部数值和各回合边界图像。它不覆盖已有输出；来源与校验结果在新目录的
`meta/conversion.json` 中。

```bash
# 仅在目标目录尚不存在时运行；使用已安装 LeRobot 的 Python 环境。
/home/robot/miniconda3/envs/lerobot/bin/python scripts/select_dataset_state.py
```

训练配置为 `configs/training/act_insert_plug_v4_with_current.yaml`，与 v4 基线保持
相同训练参数：ACT、50,000 步、batch 32、seed 1000、BF16，仅改变数据集与实验路径。
本次输出现位于 `outputs/models/act_piper_insert_plug_v4_with_current_20261026`，日志为
`logs/act_piper_insert_plug_v4_with_current_20261026.log`。模型从头训练，视觉骨干使用
同一份 ImageNet 初始化；动作仍为 7 维。部署该模型时也需提供一致的 13 维观测。

### 已有 SDK 会话导出

使用模型环境中的 Python 导出完整会话：

```bash
python scripts/export_dataset.py /path/to/session \
  --output /path/to/new/dataset --repo-id local/piper_task
```

动作来自 JSONL 中的实际目标，不使用原始预测。导出不需要 ROS 或机械臂，也不上传数据。默认不将截断或失败回合作为完整示教；显式选择规则见 `--help`。

通用训练配置采用 LeRobot 原生 TrainPipelineConfig 结构，例如 `configs/training/act.yaml`。`policy.type` 明确选择模型，dataset 指向本地转换结果。模型配置和训练超参数由 LeRobot 校验，本框架不重新实现各模型训练算法。

```bash
python scripts/train_policy.py --config configs/training/act.yaml \
  --python /path/to/lerobot/bin/python --lerobot-source /path/to/lerobot --dry-run
# 检查配置和命令后，去掉 --dry-run 才开始实际训练。
```

训练和部署使用匹配的输入名称、状态／动作顺序与单位。已有双臂模型不会被自动裁剪为单臂；不同模型所需的依赖和本地资源需要事先准备。

## π0.5/RLT 在线学习

RLT 依赖 [现有自定义 LeRobot 分支](../third_party/README.md)。在 `configs/rlt_online.yaml` 中填写 RLT checkpoint、模型解释器及 LeRobot 源码路径。若 checkpoint 保存的冻结模型或 tokenizer 路径不适用于当前机器，可覆盖对应字段：

```yaml
policy:
  overrides:
    pi05_pretrained_path: /path/to/pi05/pretrained_model
    text_tokenizer_name: /path/to/tokenizer
```

这两个资源路径使用本地绝对路径，运行时不会自动下载。RLT checkpoint 必须可用于推理；第一阶段的 `mode: rlt_training` 配置不能直接用于实机工作流。完成配置后运行：

```bash
python scripts/run.py --config configs/rlt_online.yaml --execute
```

默认使用文件采集，不启动 ROS；如需同时录包，显式选择 `record.backend: ros`。模型、人工接管和 SDK 控制路径完全相同。

`workflow.rlt.replay_windows` 默认 `checkpoint_stride`，复用原 learner 库的 `build_stride2_transitions`，按 checkpoint 的 `action_stride` 生成重叠窗口。普通重规划可以跨窗；暂停和控制权切换形成独立片段，窗口不跨这些间隔。每一步使用对应预测 chunk 的实际参考行，HUMAN 参考行替换成实际动作。未执行预测不进入回放。另可显式选择 `execution_chunks`，按不重叠的执行窗口采集。

默认重叠模式将图像和观测暂存到有大小限制的磁盘目录，运动时不额外运行冻结 VLA。回合结束、进入 HOLD 后，才重新编码真实边界、构建回放、更新并保存。默认暂存上限 8 GiB、保留 1 GiB 磁盘空间、单回合最多 10000 步，可在配置中调整。进程结束会清理该次暂存。

学习期间不能开始新回合。快照目录同时保存 `learner.pt`、`replay.npz` 与 `manifest.json`；通过 `workflow.rlt.resume` 恢复。保存的采样模式、动作契约和 checkpoint 处理器必须匹配。RLT 手动 critical gate 尚无交互命令，纯手动 gate 配置会报错；可使用明确的全程 actor 配置或训练好的自动 classifier。

## 离线重建与续训

```bash
python scripts/export_replay.py /path/to/session /path/to/new/replay.npz \
  --config configs/rlt_online.yaml
python scripts/train_online.py --resume /path/to/training/episode_000001 \
  --output /path/to/new/snapshot --updates 100 --batch-size 32 --device cuda
```

前一个入口用匹配的 RLT 模型重建实际观测边界，不更新权重；后一个只离线优化，不启动硬件。新目录原子发布，避免 learner 与 replay 保存到不同进度。实际 π0.5 推理、编码和学习耗时需在目标设备测量，模拟通过不能证明目标频率已达到。
