# 四路独立 CAN 的 ROS 主从控制

2026-10-19 起，四台 Piper 各接一个 USB-CAN，主从之间的 CAN 直连已拆开。
`main` 使用此结构；旧版保存在两个仓库的 `backup/two-can-before-four-can-20261019` 分支。

## 选择机械臂

`teleop.sh --arms both|left|right` 默认双臂；`left/right` 仅启动所选侧的主臂和从臂节点，
预检查、对齐、跟随及 `r` 复位都只操作该侧。另一侧可以断电。
录制终端同时传 `record.sh --arms left/right --output DIR`，转换时再传同侧的 `--arms`。
默认相机数量保持三路。示例见[单臂采集与部署](../README.md#选择双臂或单臂)。

## 已实物辨认的映射

| 机械臂 | 通道 | USB 物理路径 | 固件 |
| --- | --- | --- | --- |
| 左主臂 | can1 | 1-5.4:1.0 | S-V1.9-0 |
| 右主臂 | can2 | 1-5.2.3:1.0 | S-V1.9-0 |
| 左从臂 | can3 | 1-5.3:1.0 | S-V1.9-0 |
| 右从臂 | can0 | 1-5.2.2:1.0 | S-V1.9-0 |

映射通过逐台拔出 USB 适配器确认，版本通过固件查询确认。保持 USB 插在原孔；
接口编号或 USB 路径不符时，控制器拒绝启动，不能只按旧编号猜测左右。
配置入口是 `src/hardware_bringup/config/hardware.yaml` 的 `arms`（从臂）和 `leaders`（主臂）。
部署、复位及回放的从臂映射也已更新为左 can3、右 can0。

## 通信与参考实现

主臂节点读取本侧原生关节/夹爪目标，发布 ROS；对应从臂节点验证后通过 SDK 下发。
两个从臂节点分别占用自己的 CAN 控制锁，主臂原始 CAN 指令不会直接到达从臂。

参考 [kai0 AgileX DAgger](https://github.com/OpenDriveLab/kai0/tree/main/train_deploy_alignment/dagger/agilex)：
四个 CAN、四个 ROS 节点、电脑控制主臂对齐从臂、主臂切换拖动示教、ROS 转发目标。
本工作空间使用现有 pyAgxArm 的固件 profile、物理范围及看门狗。
未复制参考代码的固定中间姿态、旧版 SDK、近似角度换算常数。

## 启动

先结束已有采集、硬件、部署、复位、回放或遥操作进程：

```bash
~/AgileX_ws/prepare_can.sh --mode=teleop
~/AgileX_ws/teleop.sh --configure-only
~/AgileX_ws/teleop.sh --dry-run --duration=10
~/AgileX_ws/teleop.sh --execute
```

- `prepare_can.sh` 按配置准备全部四路 1 Mbps、正常收发。单臂独立 CAN 需要电脑提供 ACK，
  因此即使 ROS 只读取报文，也使用正常 CAN 模式，不使用旧主从共线时的 LISTEN-ONLY。
- `--configure-only` 将独立臂设为可反馈的受控模式、退出拖动示教并查询固件，不使能电机，
  不发送关节/夹爪目标；首次独立连接时原主臂可能只发启动报文，需要此初始化。
- `--dry-run` 只接收 CAN 并发布 ROS，不创建 SDK 驱动、不配置、不使能、不移动。
  如果之前未初始化且没有反馈，会看到等待/未就绪。
- `--execute` 按回车后四臂以当前位置使能保持，然后提供交互命令。
  上次退出留下阻尼停止时，只在此显式执行路径中基于新鲜反馈恢复运动状态。

交互命令输入后按回车：

| 命令 | 动作 |
| --- | --- |
| d | 暂停从臂跟随，依次把左右主臂对齐各自从臂，进入拖动示教，再开启两侧跟随 |
| p | 从臂停止跟随并保持当前实测姿态，主臂仍可拖动 |
| r | 保存并结束当前录制，暂停跟随；从臂执行原复位流程，再将主臂对齐到复位后的从臂；完成后四臂保持 |
| q / Ctrl+C | 结束本次启动的节点；从臂请求保持，示教中的主臂请求阻尼停止 |

每次 `d` 主臂会主动移动，请先放开主臂并清空路径。对齐检查新鲜反馈、目标到位和稳定时间；
不具备碰撞检测。可在 `teleop.yaml` 中显式设置安装现场验证过的六关节中间姿态，默认 null。
对齐失败、通信失联、设备故障均结束本次控制，不自动恢复。

`r` 后按回车执行采集复位。请解除插接接触、放开主臂并留出路径；开爪会释放夹持物。
程序先通过录制服务保存当前段，保存失败则不开始复位。左右从臂依次执行
`act.yaml` 中现有的开爪至 100 mm → 向上抬升 10 cm → 返回固定姿态 → 闭爪流程，
随后两台主臂依次对齐各自从臂并保持。主臂不额外执行从臂的接触脱离抬升。
固定姿态默认左右均为 `[0, 0, 0, 0, 24, 0]°`，共用 `reset.home_joints_rad`，没有另存一份坐标。
复位使用各节点已持有的 SDK 和控制锁，不启动第二个控制器；规划期间维持保持命令。
失败即停止本次会话；完成后输入 `d` 才重新跟随，夹爪接管并 READY 后再录下一段。
通过复位键结束的 `record.sh` 会保存并退出，下一段需要换目录重新启动。

进入示教时，原生夹爪输入可能回到 0，即使主臂实体夹爪刚被对齐到打开位置。
为避免瞬间闭合，从臂夹爪先保持开口；操作者把主臂夹爪输入开合到该位置（容差 5 mm）后
才接入直接跟随。随后夹爪不再受到额外逐步限速。`status.gripper_ready` 显示是否已接管。

## 跟随速度

配置为 `src/model_deploy/config/teleop.yaml`：

- 跟随目标发布和执行为 100 Hz，`sdk_speed_percent: 100`。
- `limit_follow_speed: false`：跟随阶段取消应用层目标逐步限速、已发目标与反馈的跟踪距离门限。
  SDK MOVE J 本身仍有固件规划、速度和加速度约束；100% 不是 rad/s，也不保证等同原生高跟随。
- `joint_speed_rad_s: 0.30`、`gripper_speed_m_s: 0.02`、`alignment_sdk_speed_percent: 20`
  用于保持/自动对齐；将 `limit_follow_speed` 改为 true 后也用于跟随。
- 所有模式均保留物理范围、有限数检查、反馈时效、设备状态、外部控制检测和命令看门狗。
- 遥操作参数与 ACT 部署参数独立；本次速度调整不改变 ACT 的控制速度。

## ROS 话题与采集

每侧 `{side}` 为 left/right：

| 话题 | 内容 |
| --- | --- |
| `/piper/{side}_leader/command` | 7 维绝对主臂目标：六关节 rad、夹爪 m；仅示教时发布 |
| `/agilex/teleop/{side}_leader/status` | 主臂阶段、真实 CAN 报文年龄、固件及故障 |
| `/agilex/teleop/{side}_leader/state` | 主臂受控/对齐时的新鲜实测姿态；示教中无反馈时不伪造 |
| `/agilex/teleop/{side}_follower/state` | 从臂新鲜实测姿态 |
| `/agilex/teleop/{side}_follower/commanded` | 从臂经过范围/限速处理后真正下发给 SDK 的目标 |
| `/agilex/teleop/{side}_follower/status` | holding/following/stopped、固件及 CAN 健康 |
| `/piper/{side}_bus/leader_targets` | 兼容现有转换的六关节人工目标，仅在源更新时发布并锁存 |
| `/piper/{side}_bus/gripper/target_stamped` | 同上，夹爪目标 |

首个示教目标由已对齐的实测姿态初始化，随后按主臂原生控制报文分别更新关节和夹爪。
缓存值不改写原始时间戳；原生目标可能静止不更新，通过低频只读固件查询应答验证主臂 CAN 存活。
这可检测总线/控制器失联，不等于独立证明主臂的每个关节传感器正常。

原有从臂反馈及相机话题保持原名称。录制时在一个终端运行：

```bash
~/AgileX_ws/teleop.sh --execute --cameras=all
# 输入 d，等到主从跟随开启和硬件 READY
```

另一终端使用原 `record.sh --output=...`。不要同时再运行 `start_hardware.sh`。
遥操作启动使用 record 硬件模式；四个节点就绪、关节跟随开启且两侧夹爪接管完成后才 READY。
暂停或源失联会使录制不再就绪。录完先结束录制，再暂停/退出遥操作。
新增主臂命令和 SDK 下发目标也写入录制话题清单；原转换的 action 仍是人工目标，
需要训练“约束后实际下发动作”时必须显式更换转换源，不能将两者混为一谈。

本入口属于数据采集模式，不加载模型。普通部署只需要从臂通电，并通过 `reset.sh` 仅复位从臂。
RL 人在环所需的模型控制从臂、主臂同步运动、人工中断接管与交还，以及该模式的四臂复位均待后续实现。
