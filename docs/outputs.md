# outputs 目录与文件来源

`outputs/` 按用途整理如下；手动录制的原始 bag 仍在 `bags/`，训练数据集仍在 `datasets/`，训练控制台日志仍在 `logs/`。

```text
outputs/
  models/                       # 模型、checkpoint、优化器与训练状态
    act_piper_insert_plug_20261016/
    act_piper_insert_plug_v3_all_20261017/
    act_piper_insert_plug_v4_20261025/
    act_piper_insert_plug_v4_with_current_20261026/
    pi05_piper_insert_plug_v4_005000/
  reports/
    datasets/
      piper_insert_plug_v4/                  # 7 维数据集检查与转换日志
      piper_insert_plug_v4_with_motor_state/ # 19 维数据集检查、转换与独立核验
    deployment/
      act_insert_plug_v4_right/              # 右从臂 ACT 050000 实机运行记录
  sessions/
    collect_right/              # 之前的自动采集会话
    piper/                      # 今后 files/ros 会话的默认输出位置，录制时创建
  training/                     # 历次训练的启动脚本、退出状态和锁文件
  archive/                      # 历史指针与本次目录迁移表
```

## 原来根目录的 JSON 从哪里来

这些文件是在之前的数据转换和核验过程中生成的，现按数据集归档：

| 现在的位置 | 来源及用途 |
| --- | --- |
| [v4/inspection.json](../outputs/reports/datasets/piper_insert_plug_v4/inspection.json) | 原名 `insert_plug_v4_inspection.json`。`scripts/convert_lerobot.sh` 调用 `scripts/bag_to_lerobot.py --inspect --report`，检查 bag 中的状态、人工动作、双相机图像和时间同步。 |
| [motor_state/inspection.json](../outputs/reports/datasets/piper_insert_plug_v4_with_motor_state/inspection.json) | 原名 `insert_plug_v4_with_motor_state_inspection.json`。同一转换器使用增加电流和估算力矩的配置，额外检查逐电机反馈时间戳。 |
| [motor_state/independent_validation.json](../outputs/reports/datasets/piper_insert_plug_v4_with_motor_state/independent_validation.json) | 原名 `insert_plug_v4_with_motor_state_independent_validation.json`。临时脚本 `/tmp/audit_insert_motor_dataset.py` 独立比对原始 bag、新数据集与旧数据集，报告内记录了该脚本路径和校验方法。 |

三份报告对应 105 段示教、18,832 帧。独立核验报告的 `status: passed` 记录了当时的核验结果：19 维状态／7 维动作匹配、原 7 维数据集与视频保持不变，电机采样年龄最大约 97.8 ms。本次整理只移动报告，没有重新进行全量数据核验。

同目录的 `conversion.log` 是全量转换日志；`inspection.log` 是检查日志；`smoke.log` 是单个 bag 转换测试日志。今后的检查报告也可直接放到这里，例如：

```bash
bash scripts/convert_lerobot.sh --inspect \
  --report outputs/reports/datasets/piper_insert_plug_v4/inspection_new.json
```

`--report` 使用新文件名，避免覆盖已有记录；检查其它任务时同时指定对应的 `--config` 或 `--input`。

## 模型目录中的 JSON

checkpoint 内的 JSON 与上述检查报告用途不同，应和权重放在一起：

| 文件 | 用途 |
| --- | --- |
| `model.safetensors` 或分片权重 | 模型参数 |
| `config.json` | 模型类型、结构和输入输出维度 |
| `policy_preprocessor.json`、`policy_postprocessor.json` 及其 `.safetensors` | 预处理、归一化和动作反归一化 |
| `train_config.json`、`training_state/` | 当时的训练设置，以及续训所需的进度、优化器等状态 |

当前右从臂 ACT 部署使用 `models/act_piper_insert_plug_v4_20261025/checkpoints/050000/pretrained_model`；电流实验模型在 `models/act_piper_insert_plug_v4_with_current_20261026`，需要与其匹配的 13 维状态输入。

π0.5 文件夹是导入模型。其保存的训练配置仍记载远端路径和 `local/piper_insert_plug_v1`，本次保留原名和原始记录，不能仅根据文件夹名认定它来自当前 v4 数据集。

## 运行记录与旧文件

- [reports/deployment/act_insert_plug_v4_right](../outputs/reports/deployment/act_insert_plug_v4_right)：`preflight.json` 为只预测、不执行的检查；`deployment_report.json` 为 30 秒实机运行结果；`execution.jsonl` 保存目标、反馈和事件。旁边的 NumPy 文件是预检观测与预测动作。
- [sessions/collect_right](../outputs/sessions/collect_right)：之前的两次自动采集会话，保留 bag、图像、轨迹和 manifest。两次均因停止请求标记为 `failed`，不能视为已验证完整的示教。
- [training](../outputs/training)：已结束训练的 `launch.sh`，以及电流实验的 `status`、`lock`。空 `lock` 文件本身不表示仍有训练进程。
- [archive/can-fix-latest.txt](../outputs/archive/can-fix-latest.txt)：历史 CAN 测试目录指针，所指目录已经不存在，仅保留作历史记录。

## 迁移与后续使用

本次没有删除模型或数据。所有原路径与新路径对应关系保存在 [reorganization_20261026.tsv](../outputs/archive/reorganization_20261026.tsv)。现用部署、训练配置和启动脚本已更新，`scripts/run.py --config ...` 的命令不变。

checkpoint 的原始训练配置、历史报告和 manifest 内容原样保留，因此其中可能出现迁移前的绝对路径。部署按现用 YAML 中的新 checkpoint 路径加载；使用保存的 `train_config.json` 续训时，要显式覆盖 `--output_dir`，例如只预览命令：

```bash
python scripts/train_policy.py --resume --dry-run \
  --config outputs/models/act_piper_insert_plug_v4_20261025/checkpoints/050000/pretrained_model/train_config.json \
  --python ~/miniconda3/envs/lerobot/bin/python --lerobot-source third_party/lerobot \
  --output_dir="$PWD/outputs/models/act_piper_insert_plug_v4_20261025" --steps=60000
```

确认训练计划后再去掉 `--dry-run`。新建实验仍应使用新的输出目录。
