# ACT 本地训练

[返回工作空间 README](../README_zh.md) · [数据转换](../src/data_collection/README.md#bag--lerobot) · [模型部署](../src/model_deploy/README.md)

## 开始训练

使用 Conda `lerobot` 和可编辑安装的 `third_party/lerobot`。训练不启动 ROS、不控制机械臂、不上传模型。输入为三路 640×480 RGB＋14 维从臂状态，输出同顺序的 14 维绝对关节/夹爪目标；电流、估算力矩等附加列不参与这些 ACT 配置的模型输入。

先按[采集与转换说明](../src/data_collection/README.md)准备 `bags/insert_plug_v2/task_0001` 等演示，并转换为 `datasets/piper_insert_plug_v2`。确认没有原任务占用资源，使用新的输出目录启动：

```bash
~/AgileX_ws/training/train_act.sh \
  --config_path="$HOME/AgileX_ws/training/act.yaml" \
  --dataset.repo_id=local/piper_insert_plug_v2 \
  --dataset.root="$HOME/AgileX_ws/datasets/piper_insert_plug_v2" \
  --output_dir="$HOME/AgileX_ws/outputs/act_insert_plug_v2_new_run"
```

脚本未指定配置时加载 [act.yaml](act.yaml)。它保留旧批次的数据集和输出路径，训练新批次须显式覆盖 `dataset.repo_id`、`dataset.root` 和 `output_dir`，并核对划分比例、batch、步数与保存频率。上述命令沿用基线的其他训练参数；已有 v3 实验的参数另见下文。

脚本自动隔离 ROS Python 和动态库路径，并要求 CUDA 可用。默认离线运行；本机已缓存 ImageNet ResNet18 权重，换机器需先准备该权重及依赖。训练结束会输出 `End of training`；退出日志查看终端不会停止训练。

## 检查点与恢复

每次实验的 `output_dir/checkpoints/` 保存模型与训练状态。`pretrained_model/` 包含部署所需的模型、配置和归一化处理器；`training_state/` 包含恢复训练所需的优化器、步数与随机状态。只保留模型文件不能完整恢复训练。

恢复前确认旧训练进程已经退出，使用该次实验完整检查点中的 `train_config.json`，并显式保留中途调整过的参数。下面历史记录中的恢复命令只对应相应实验，不能直接用于新的 v2 或 v3 任务。

`last` 仅指最新检查点，不保证验证损失最低；没有验证集的实验不能按验证损失选优。部署前显式更新 [act.yaml](../src/model_deploy/config/act.yaml) 的 `checkpoint`，核对数据批次、归一化处理器、关节顺序和动作定义。训练脚本不会自动修改部署配置。

## 实验记录

以下按实验日期保存既有路径与参数。它们记录当时的训练安排，进程是否仍在运行需在本机检查。

### insert_plug_v3 全量训练（2026-10-17）

该轮配置为 [act_insert_plug_v3_all.yaml](act_insert_plug_v3_all.yaml)，按用户要求使用全部示范训练，关闭验证。该轮重新初始化模型及优化器（视觉骨干仍用 ImageNet 初始化），采用新数据集的归一化统计。先前保留 2 条验证的 v3 任务已停止，日志和配置保留。

- 数据：`datasets/piper_insert_plug_v3`；`data_0001`–`data_0020` 全部用于训练（20 条、9072 帧），不划分验证集。
- 参数：batch 32、28750 步、BF16、学习率 1e-5、chunk 100；`dataset.eval_split: 0.0`、`eval_steps: 0`、`env_eval_freq: 0`。每 1250 步保存，输入仍为三路 RGB＋14 维从臂状态。
- 环境：本机 RTX 4090、Conda `lerobot`、可编辑安装的 `third_party/lerobot`。
- 输出：`outputs/act_piper_insert_plug_v3_all_20261017/checkpoints/`。没有验证损失供选优，`last` 表示最新检查点；部署配置不会自动切换到新模型。

查看后台训练：

```bash
tail -f ~/AgileX_ws/logs/act_piper_insert_plug_v3_all_20261017.log
```

进程号及启动参数保存在同目录 `.pid` 和 `.run.json`。退出 `tail` 不会停止训练；电脑需保持开机且不休眠。若该任务仍在运行，不要重复启动。手动启动同参数实验须使用新的输出目录：

```bash
~/AgileX_ws/training/train_act.sh \
  --config_path=~/AgileX_ws/training/act_insert_plug_v3_all.yaml \
  --output_dir="$HOME/AgileX_ws/outputs/act_insert_plug_v3_new_run"
```

### 基线与续训（2026-10-16）

使用 `third_party/lerobot` 可编辑安装和 Conda `lerobot` 环境，训练配置见 [act.yaml](act.yaml)。输入数据为 `datasets/piper_insert_plug`；训练不启动 ROS、不控制机械臂、不上传模型。

模型输入：三路 640×480 RGB＋14 维从臂状态 → 14 维绝对关节/夹爪目标；电流力矩等附加列不参与本次模型输入。ACT 默认 ResNet18（ImageNet 初始化）、chunk size 100、学习率 1e-5。单张 RTX 4090，BF16 混合精度。`act.yaml` 保留初始基线（batch 8、100000 步），该轮续训调整见下文。

按完整 episode 划分：`task_0001`–`task_0018` 训练（8396 帧），`task_0019`–`task_0020` 验证（866 帧）。验证损失是离线指标，不是实机插接成功率。

#### batch 调整

第 5000 步已保存，然后测试了 batch 16/32/40。稳定区间实测分别为约 96/94/显存不足，单位为样本/秒；batch 8 原速度约 93 样本/秒。batch 32 的 PyTorch 显存分配峰值约 19.9 GiB，batch 40 超出 24 GB 容量。显存占用增加，但速度提升很小。

正式训练从 `005000` 的模型及优化器状态恢复，改用 **batch 32，终止步数 28750**，每 **1250** 步验证并保存。原计划的约 80 万次样本采样保持不变：`5000×8 + (28750−5000)×32 = 800000`。模型更新次数减少，不能把相同样本数当作完全相同的优化过程。

训练日志中的 `smpl/epch` 可能按当前 batch 乘累计 step 显示，不适合作为跨 batch 阶段的实际累计采样量；阶段记录见 `logs/act_batch_adjustment.json`，速度测试见 `logs/act_batch_benchmark.json`。最近检查点里的 `train_config.json` 才是该检查点的续训配置。

#### 任务与日志

2026-10-16 启动的正式训练：

```bash
tail -f ~/AgileX_ws/logs/act_piper_insert_plug_20261016.log
```

启动元数据与进程号保存在同目录 `.run.json` 和 `.pid` 文件。训练结束会输出 `End of training`。退出 `tail` 不会停止训练，电脑需保持开机且不休眠。

模型目录为：

```text
outputs/act_piper_insert_plug_20261016/checkpoints/
├── 005000/
│   ├── pretrained_model/    # 模型、配置、归一化处理器
│   └── training_state/     # 优化器、步数、随机状态，可续训
├── ...
├── 28750/                  # 实际目录编号由 LeRobot 补零规则决定
└── last -> 最新检查点
```

`005000` 已存在，后续每 1250 步保存。`last` 表示最新，不保证验证损失最低；可根据日志选择具体步数。`outputs/act_smoke_20261016` 是已完成的 8 步功能测试，不是正式训练结果。

#### 恢复该次续训

若该次训练中断，确认原训练进程已退出，再从其最新完整检查点恢复：

```bash
~/AgileX_ws/training/train_act.sh \
  --config_path="$HOME/AgileX_ws/outputs/act_piper_insert_plug_20261016/checkpoints/last/pretrained_model/train_config.json" \
  --resume=true --batch_size=32 --steps=28750 --save_freq=1250 --eval_steps=1250
```

恢复将使用检查点配置和优化器状态，上述 CLI 显式保留此次 batch/步数调整（即使最新检查点仍为旧的 `005000`）。只保留 `pretrained_model` 可以用于推理，但不能完整恢复优化器状态。部署时必须同时保留归一化处理器与关节顺序，运行方法见 [model_deploy](../src/model_deploy/README.md)。
