# RL-Token 论文与当前实现核对

核对日期：2026-10-20。此次只新增审查文档，没有修改算法、训练配置或机器人控制代码。

**2026-10-20 实验决定：用户选择先按当前实现训练，保留以下差异，暂不修改算法。**
后续如效果不佳，再逐项改动并与当前基线比较。下文修改建议是待评估事项，不是本次提交已完成的修复。
RLT 仓库内同步保存本报告和 `examples/pi05_rlt/BASELINE.md`，避免换机器后丢失记录。

**结论：当前是基于第三方 π0.5 项目的 RLT 训练后端移植，具有论文的主要算法结构，但不应称为论文完整系统或实验配置的严格复现。**
此前的测试证明代码路径能运行、保存和恢复，不能证明与论文逐项等价，更不能证明实际任务收益。

## 核对依据

- 原文：[arXiv:2604.23073v2 PDF](https://arxiv.org/pdf/2604.23073v2)，2026-04-30 版本，13 页；核对了公式、图 2、Algorithm 1、正文和附录 B/C。
- [原文 HTML](https://arxiv.org/html/2604.23073v2)，便于跳转阅读。
- 当前库：`third_party/lerobot`，分支 `test/pi05_rlt`；核对时算法文件尚未提交，当前版本以该分支 Git 提交为准。
- 第三方实现：[RL-Token-Pi05-open](https://github.com/Zhengsw03/RL-Token-Pi05-open/tree/cccf949c782266f3dab227ec4265a6360e0511e2)，本地参考副本的提交为 `cccf949c782266f3dab227ec4265a6360e0511e2`。

下文区分：明确结构差异、硬件/实验适配、系统缺失、论文未充分说明的实现选择。第三方代码不能作为论文细节的最终依据。
所有代码行号对应本次审查时的工作区。

## 与论文主要结构对应的部分

| 项目 | 论文位置 | 当前代码与结论 |
| --- | --- | --- |
| 最后层图像特征、固定任务时丢弃语言位置 | IV-A，图 2、脚注 1 | `vla_compat.py:181–199` 先做 Transformer forward，再截取图像位置；结构对应。当前 π0.5 将离散状态放入文本，因此也一并丢弃这些位置；输入语义存在骨干差异。 |
| 追加可学习 RL token，取末位置 | 式 (1) | `modeling_pi05_rlt.py:82–95` 对应；没有误用均值池化。图 2 的 2048 维也与默认值一致。 |
| 停止梯度、右移教师输入、因果重建 | 式 (2) | `modeling_pi05_rlt.py:147–175, 813–831` 对应。使用带因果掩码的 `TransformerEncoder` 实现自回归解码，本身不构成偏离。输出投影另见下文。 |
| RL 阶段冻结骨干和 token 模块 | IV-B | `modeling_pi05_rlt.py:435–447` 对应；Actor/Critic 可训练，目标网络冻结。 |
| 直接输出动作的固定方差 Gaussian Actor | 式 (4) | `RLTChunkActor` 对应，没有把 Actor 写成动作残差加法。固定 std 的具体数值不能由论文确认。 |
| 状态/参考动作条件化 Actor，状态/执行动作条件化 Critic | IV-B | Actor/Twin-Critic 拼接关系对应；本体状态内容见差异项。 |
| 逐步折扣、chunk 末边界 bootstrap | 式 (3) | `learning.py:25–38, 62–81` 对应。有效长度为 n 时使用 gamma^n，终止时屏蔽 bootstrap；不是把整个 chunk 错当一步折扣。 |
| Q 目标与动作参考正则 | 式 (5) | `learning.py:85–109` 结构对应；参考 dropout 不会清零 BC 目标。损失归约尺度见下文。 |
| 人工动作替换回放参考 | Algorithm 1，第 11 行 | `replay.py:236–239` 对明确标记 HUMAN 的行执行替换；`learning.py:99–103` 对 HUMAN/MIXED 使用实际动作做 BC。这是数据处理能力，尚不是实机接管。 |
| 连续动作执行与重叠回放 | 第 V 节 | 默认 C=10，回放起点 stride=2；推理 `modeling_pi05_rlt.py:1020` 取前 C 个连续动作，没有隔行执行。交叉窗口参考定义仍有待明确，见下文。 |
| 双 Q 的最小目标、50% 参考 dropout | 附录 B | `learning.py:66–68, 93–95` 对应。 |
| Critic:Actor = 2:1、UTD=5 | 第 V 节 | 学习器默认值对应；UTD 预算有 API，但没有在线调度器把它落实为真实采样/更新节奏。 |

## 已知差异（当前实验基线暂不修改）

### 1. 解码器默认没有可学习输出投影：明确结构差异

论文 IV-A 描述解码器后存在输出投影 h_phi。当前 `modeling_pi05_rlt.py:130`：

```python
self.pred_head = nn.Linear(d, config.vlm_hidden_dim) if d != config.vlm_hidden_dim else nn.Identity()
```

默认两个维度都是 2048，因此实际使用 `Identity`。维度相同只能说明投影不是维度转换所必需，不能说明可学习投影可省略。
建议把该层显式配置为可学习 Linear，再重训 Stage 1。新增参数会改变检查点结构，旧权重的严格加载不能原样保留；图像特征缓存本身不因此失效。
这一条件分支来自第三方源码，当前移植保留了它。

### 2. 当前策略入口只取均值：不能直接用于在线探索

`modeling_pi05_rlt.py:626–628, 1052–1057` 从 Actor 取得 `mean, std` 后只返回 mean。
`predict_action_chunk()` 和 `select_action()` 都沿用这一路径，即使配置是 `mode=online_rl`。
训练损失内确实会采 Gaussian 动作，但这不等于机器人收集数据时也进行了随机探索。

对最终评估使用均值可以是合理工程选择；若未来在线采集器直接调用这个接口，则与 Algorithm 1 的训练采样行为不同。
应明确拆分“评估均值动作”和“训练采样动作”，同时返回回放边界信息。随机动作还要经过实际执行约束，并记录最终执行值。

### 3. 本体状态只有位置，速度未接入；末端位姿也未独立支持

附录 B 提及位置和速度；第 VI-A 节按任务区分关节位置与末端位姿。
当前默认 `state_dim=14`，`_infer_window()` 在 `modeling_pi05_rlt.py:1040–1043` 只读 `observation.state`。
本工作区该字段由双侧关节位置和夹爪宽度组成，见 `src/data_collection/config/lerobot_conversion.yaml:125–136`。
独立的 `observation.motor_velocity_rad_s` 虽已声明，但没有接入 RLT 状态、缓存或 learner。

这会影响接触阶段的可观测性。不能只把 `state_dim` 改大：`configuration_pi05_rlt.py:278–283` 将其与骨干的状态特征维度绑定，且还受 `max_state_dim` 限制。
建议独立定义骨干状态与 RL 本体状态，例如 `vla_state_dim`、`rl_proprio_dim`、`rl_proprio_features`，再确定各字段单位、时间戳和归一化统计。
速度维度也不能未经测量就默认等于位置维度，尤其是夹爪速度。

### 4. 动作空间与控制频率属于明确的实验适配

论文实验使用 50 Hz，附录 C 说明统一比较设置中的 delta 动作空间。
当前 `precompute.py:45–49` 和 `configuration_pi05_rlt.py:273–277` 明确拒绝 `use_relative_actions=True`。
现有 AgileX 数据转换配置记录的是绝对关节目标，默认数据/ACT 推理频率为 15 Hz。

RLT 尚无自己的真机运行频率配置，所以不能断言它已经在 15 Hz 运行；但如果沿用现有设置，C=10 对应约 0.667 秒，而 50 Hz 下是 0.2 秒。
这会改变反应时间、参考轨迹跨度、折扣的真实时间含义、限速约束和探索噪声效果。
对关节绝对动作使用 RLT 可以是合理的 Piper 变体，但不能称为相同实验条件。
若要支持 delta，需要一起处理 SFT 数据、归一化、参考/人工动作、回放表示、动作累积与实际下发，不能仅取消配置检查。

### 5. 在线系统仍缺失：不只是少一个人工接管按键

`train_stage2.py:21–22, 40–52` 从固定 NPZ 加载 replay 后重复更新；没有与机器人交互的采集循环。
当前没有实现基于 VLA 的 N_warm 环境步预填充、关键阶段内的采样筛选、每步观测快照、成功/失败标注终止、异步 rollout/learner、权重发布与一致保存。
`learner.learn()` 只提供按新增 transition 数计算更新预算的同步 API。

论文第 V 节中的人工纠正属于可选监督方式，但关键阶段任务定义、奖励和真实在线交互不能由离线优化脚本代替。
后续应明确区分四种控制状态：BASE、RL、HUMAN、STOP/RESET。VLA→RL 阶段切换与模型→人控制权接管是两种事件。
当前 ROS 控制层尚未把这些状态、队列清除、主臂同步和四臂复位连接起来。
回放的来源字段也不自动保证只训练关键阶段，必须由采集器实现。

## 需要声明或进一步确认的实现选择

### 6. TD 目标使用 target Actor，与公式字面不同，但不能直接判为错误

式 (3) 写的是从 pi_theta 采样，明确标出 target Critic psi'；同时正文说明参考 TD3。
当前 `learning.py:66` 使用 `actor_target`，并在 `learning.py:221–222` 对 Actor/Critic 一起做 Polyak 更新。
该做法与所参考第三方 runner 一致，也与常见 TD3 工程实现相容。
论文未展开 Actor target 的细节，因此结论是“公式字面差异/未充分披露项”，而不是已证明算法错误。
建议增加目标 Actor 策略配置，记录选择；若追求公式逐字对应，可比较当前 Actor 与 target Actor 的版本。
同样，不应擅自把标准 TD3 的全部附加机制都视为论文已经明确采用。

### 7. Actor 正则用平均误差，beta 数字不能照搬原始平方范数

`learning.py:103` 对动作维求 mean，再对有效步求 mean。对完整 C×d 动作，这相当于平方范数除以 C*d。
因此当前有效正则系数相对于未归约范数为 `bc_weight / (C*d)`。

举例：C=10、d=14、每维误差为 1 时，当前 BC=1；未归约平方范数=140。
当前 `bc_weight=10` 时 BC 项为 10，而直接对未归约范数乘同一数字会得到 1400。
这属于可以通过系数换算表达的尺度选择，并不证明均值损失无效；但会影响与 Q 的相对权重，尤其是单/双臂切换时。
论文没有提供可直接核实的 beta 数值，不能声称当前 10 就是论文参数。
Stage 1 同样使用平均 MSE 而非原始 token 误差之和；优化目标的结构对应，但梯度尺度与裁剪关系需记录。

### 8. 重叠回放的参考 chunk 可能拼接自不同边界

`replay.py:352–354` 会用每步 `StepRecord.ref_row` 覆盖边界快照里的非人工参考。
例如起点 t=8、C=10 的窗口，可能把 t=0 那次规划的末两行与 t=10 那次规划的前八行拼在一起。
它就不一定等于由窗口起点 x_8 采样出的单个 VLA chunk。

式 (5) 将参考与当前状态条件化；但论文没有给出重叠窗口参考的完整生成伪代码。
因此，这是需要明确的语义选择，不能仅凭第三方代码就宣称唯一正确。
建议对正常重叠窗口保留起点状态对应的参考，人工行按监督规则替换，并记录 reference_origin；
若保留逐步拼接做法，应明确标成变体并比较训练/推理分布差异。

### 9. 分层回放默认值甚至与第三方实际 runner 不一致

当前 `ReplayBuffer` 默认 `sample_strategy="stratified"`，配额设置为 recent/warmup/human=0.4/0.3/0.2。
这些不是论文给出的采样比例。第三方 `scripts/train_rlt_stage2_pi05.py:2112–2119` 实际创建 buffer 时显式使用 `uniform`。
此前整合保留了类的分层默认值，却没有沿用 runner 的 uniform 设置；这是本次审查确认的一处移植默认行为差异。
采样方式保存在新的 NPZ 中，训练脚本加载时会沿用它，因此应在真实训练前显式选定并写入实验配置。
可以使用 uniform 作为无额外采样加权的基线，但论文没有把具体回放采样器充分公开，不能据此宣称是作者实现。

### 10. 自动切换分类器是替代工程方案

论文第 V 节讨论由 VLA 学习切换时机。当前 `train_critical.py:63–73` 训练独立 MLP，输入为 z_rl 和归一化本体状态；
这不是微调 VLA 去预测切换信号。
此外，`_fill_window()` 在未提前翻转时，会先用上一窗口 z/current proprio 检测，再用当前 z 检测，
所以检测器更新次数不总等于 chunk 次数；平滑和 min_on_chunks 的时间解释需重新明确。
手动切换可以先用于在线实验，不必把自动检测作为核心算法完成的前置条件。

### 11. 多个数值只有第三方/本地依据，论文未给出

包括 encoder/decoder 各 4 层、8 heads、dropout=0.1、std=0.05、gamma=0.99、tau=0.005、
学习率、batch size、beta=10、TD 目标 ±2 裁剪、Critic 梯度阈值 10、分类器阈值等。
不能把源代码注释里的 “paper” 当作这些数值已由论文证实的证据。
当前 Stage 1 CLI 默认 30000 步，而附录 B 报告的是 2000–10000 步；这是训练配方差异，不能仅按步数判断结果优劣。
Actor/Critic 的 2 层 256 默认值有论文依据；更大 3 层 512 是另一任务配方，已有配置能力可表达，不能一概套用。

### 12. π0.5 骨干和分步 SFT 属于可接受但需声明的适配

论文使用 π0.6；当前只允许 `PI05Config`。相同 token 维度不意味着骨干、特征与行为先验相同。
这属于本次任务选用第三方 π0.5 实现的明确范围，不能通过少量配置变更实现骨干等价。

当前先准备任务 SFT 权重，再冻结缓存训练 token，对应论文允许的冻结骨干路线；
不支持联合 SFT 不能直接判定违反算法。若声称复现论文实验的联合适配过程，则还缺对应训练入口。
若以后做联合 SFT，旧特征缓存不能继续跨骨干更新复用。

## 本次验证证据与边界

重新运行 `tests/policies/pi05_rlt`：**35 passed**。没有下载模型权重、没有启动机器人。
另在 `/tmp/rlt_paper_audit_probes.py` 运行了小模型检查，输出保存在 `/tmp/rlt-paper-audit-probes.json`：

```json
{
  "decoder_output_projection": "Identity",
  "actor_linear_layers": 2,
  "bc_normalized": 1.0,
  "bc_raw_squared_norm": 140.0,
  "actor_loss_beta10_zero_q": 10.0,
  "td_target_actor_calls": {"actor": 0, "actor_target": 1},
  "consecutive_predict_chunks_equal": true,
  "predict_chunk_std": 0.0,
  "configured_gaussian_std": 0.05,
  "default_replay_strategy": "stratified"
}
```

确定性动作检查使用固定边界输入和零 Actor，用于确认策略入口没有额外采样；不是声称随机 VLA 在真实观测下总输出相同动作。
这些检查与源码共同支撑上述判断；没有验证真实收敛、收益、GPU 延迟或真机安全接管。

## 如需迭代时的建议顺序

1. 先确定实验契约：π0.5 变体、单/双臂、关节/末端动作、absolute/delta、控制频率与 C、RL 本体特征。
2. 在正式 Stage 1 前补输出投影，明确损失归约；区分骨干状态与 RL proprio。此时修改检查点结构成本最低。
3. 显式配置探索/评估模式、目标 Actor 选择、回放采样、超参数和重叠窗口参考来源。
4. 接入 ROS 在线状态机、warmup、关键阶段筛选、每步观测/执行记录、成功失败标签和异步 learner；再验证人工接管和复位。
5. 先做小规模真实数据与权重检查，再做关键阶段短实验。分别比较 VLA、仅 BC、完整 RLT，避免把接口能运行当作 RL 有收益。

原有 `examples/pi05_rlt/README.md` 与代码中的对齐说明，应结合本报告理解；本轮没有静默修改其旧表述或实现。
