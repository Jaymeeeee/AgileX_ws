# 直接 SDK 控制

主要命令在 [`hardware/robot.py`](../piper_control/hardware/robot.py)。每个启用的机械臂各有一个 SDK 连接。`robot.arms` 选择左侧、右侧或两侧；所选侧的 `follower` 必须为 `true`，`leader` 默认是 `false`。普通单臂部署只连接一台从臂，双臂部署连接两台；需要人工接管时再设置相应侧的 `leader: true`。未被 `arms` 选中的侧不会连接。控制层将关节 rad、夹爪 m 的目标原值交给 SDK。

## 命令顺序

模型控制的核心调用如下，`arm`、`gripper` 是公开 SDK 创建的对象：

```python
arm.connect()
arm.set_follower_mode()
while not arm.enable():
    time.sleep(0.02)               # 框架中另设使能超时并响应停止
arm.set_speed_percent(100)
arm.move_js(joints_rad)
gripper.move_gripper_m(value=width_m, force=1.0)
# 完成本次控制后
arm.electronic_emergency_stop()
arm.disconnect()
```

框架向所选侧的从臂发送目标；该侧配置了主臂时，也向主臂发送同一目标，以供后续人工接管。没有配置的主臂不连接、不使能、不发送命令，也不创建主从转发器。SDK 的 `move_js` 自行设置关节运动模式；本框架不改动 SDK 内部的参数验证、限位或编码行为。模型状态和动作始终只按从臂组织，单臂 7 维、双臂 14 维。

SDK 的 `enable()` 不会阻塞等待电机使能成功。框架在首次发送后，每 20 ms 读取全部所选臂的驱动反馈，并对尚未完成的设备重发使能命令；所有六关节反馈时间戳均晚于该设备首次使能发令，且使能位均为 true，才允许发送初始位置目标。`robot.startup_enable_timeout_sec` 默认 10 秒，超时会列出每臂使能位、缺失或旧反馈以及可用的机械臂状态，随后停止并断开。

自动人工启动先检查已收到的驱动故障，将配置中的主臂切回 follower 模式，再等待所选设备的机械臂状态和六关节驱动反馈可用，任一设备有驱动故障则在复位前报告并退出。仅对 `arm_status == 1`（电子停止）的设备调用 SDK `reset()`；正常设备不会复位。复位会使关节失能，因此启动前应放稳机械臂。对实际复位的设备，等待六关节均收到发令之后的失能反馈，才允许进入使能阶段；不能用复位前缓存中的 true 判定使能成功。初始反馈等待与复位确认各使用 `startup_enable_timeout_sec` 作为独立期限。使能和对齐期间发现驱动故障会停止，对齐时不自动重新使能。

对齐失败的 `driver_faults` 按关节列出驱动故障，例如 `joint_5` 下的 `driver_overcurrent`（过流）、`driver_error_status`（驱动错误）、`stall_status`（堵转）。这些反馈说明驱动报告了故障，但不能单凭故障位确定机械原因；出现这类报警时先排查原因，不通过延长超时或反复清错启动来继续运动。

人工接管要求每个所选侧都配置主臂，调用为：

```python
leader.set_leader_mode()
forwarder.activate()              # 主臂 CAN 命令直接转发到从臂
# 恢复模型时
forwarder.deactivate()
leader.set_follower_mode()
```

模式命令后固定暂停 `robot.mode_settle_sec`，默认 0.1 秒，不轮询状态确认。切换时清空旧模型动作队列，新的模型动作从当前 observation 计算。转发器只路由 `0x151`、`0x155`、`0x156`、`0x157`、`0x159`，字节原样传递；排除 SocketCAN 本机回环和不属于这条路由的帧。

CAN 准备脚本同时应用 `robot.can_tx_queue_len`（默认 64 帧）。发送出现 `ENOBUFS`（105）或 `EAGAIN` 时，转发器保持当前帧内容和顺序，在一个总计 10 ms 的窗口内重试，每次间隔最多 0.5 ms；成功发送后才将该帧计入人工动作。它不增加软件动作队列，也不无限重试。超过期限或出现其它 CAN 错误仍停用转发并向主循环报告；暂停需等待当前短时发送结束。这个期限仅约束重试，不保证内核队列中的帧在 10 ms 内被硬件执行。

纯人工 `collect/teleop` 设置 `workflow.auto_start: true` 时，检查反馈并按需恢复电子停止，然后调用与模型部署相同的 `Controller.start()`：使能、读取从臂当前姿态，通过 `robot.command()` 向主从臂各发送一次 `move_js()` 及夹爪目标。之后仅观察反馈，不重发目标、不切换 J 模式。所有主从臂六关节需保持使能且无驱动故障，两侧位置反馈须可用且数值有限。全部所选主从臂在 0.8 秒新反馈窗口内，每个关节的位置范围不超过 0.002 rad、夹爪不超过 0.001 m，即视为运动稳定；同一缓存时间戳不能推进窗口。这个判据不要求主从精确对齐，也不保证速度严格为零或反馈严格同步。

主从当前姿态差 `pair_errors` 和各臂相对初始目标的偏差 `target_errors` 均只用于诊断，不以位置残差阻止进入人工控制。`robot.startup_alignment_timeout_sec` 默认 10 秒，从初始目标提交后开始计算；运动未稳定或缺少可用反馈时会超时停止，并列出主从差和每臂相对目标的误差，Ctrl+C 可以中止。等待期间出现驱动故障或使能丢失仍会停止。稳定后自动进入同一个 `enter_human()`，切换主臂为 leader 并开启转发。

模型部署中的人工接管没有这段启动运动稳定等待；操作员可以在 HOLD 或 POLICY 时主动切换 HUMAN。[SDK 文档](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md#joint-motion-follower-mode--move_js) 明确区分 JS 与 J 的控制特性，因此采集不再使用专有的 J 对齐分支。右侧手动采集进入 HUMAN 后还会提示等待控制数据；首次收到人工动作时显示 `人工 action 数据已就绪`。

## 已移除的框架逻辑

- 运行时人工接管的对齐检查与反馈跟踪误差检查。
- 软件关节／夹爪限位、速度裁剪、内向量化、模型目标到位确认。
- 反馈／图像年龄检查、独立 CAN 状态监视、模式切换确认与看门狗。
- 固件回读、额外 USB 身份核验、跨进程通道锁和 SDK 发送函数拦截。

保留的是基本数据格式、维度和单位处理、模型／人工控制切换、显式停止、资源关闭和异常报告。状态来自 SDK 最新缓存，原始采集时间另行记录；数据可用不表示数据足够新。

模型 `action` 表示提交给 SDK 的目标，并非底层发送或到位确认。人工 `action` 来自转发 CAN 字节的解码值。逐步记录中的 `action_evidence` 分别为 `sdk_target` 和 `forwarded_can_target`。

## 状态、动作与 ROS 采集

从臂的 `joint_states` 来自 SDK 状态反馈；主臂处于 follower 模式时也可读取同类反馈。切为 leader 模式后，主臂通常只输出控制帧，程序使用 `get_leader_joint_angles()` 和夹爪控制数据发布 `leader/command`。这份数据表示主臂给出的目标，不能当作新的实测反馈。电机与驱动诊断若来自 SDK 缓存，会保留原始反馈时间戳，不以当前时间冒充新数据。

人工动作链路为：主臂 `0x155/0x156/0x157` 关节控制帧和 `0x159` 夹爪控制帧 → CAN 转发器组成完整目标 → HUMAN 工作流发布 `action` 和 `action_metadata` → 独立录包器保存。没有完整控制帧时不会生成一份虚构动作，也不会用从臂状态补齐。因而缺少 `action` 时应先区分上游没有动作、ROS 没有发布和录包未选中三种情况，不能仅凭 bag 有图像或终端显示 HUMAN 就认定示教数据完整。

相机采集配置与录包清单分别控制数据产生和保存。`collect.yaml` 默认选择三台相机，右侧子配置 `collect_right.yaml` 仅选择顶部 `cam_h` 和右腕 `cam_r`；所选相机均开启 RGB、深度和对齐深度；每个启用的流都发布 `image_raw` 与 `camera_info`。手动录包从 [`configs/recording/topics.yaml`](../configs/recording/topics.yaml) 读取必需／可选 topic，`python scripts/record.py --config configs/collect_right.yaml --list-topics` 可查看当前设备对应的清单。录制结束后缺少必需 topic 会标记 `incomplete` 并列出缺项，保留 bag 供排查；这个检查不额外发送机械臂命令。

## CAN 配置

Linux 分配的 `can0` 等名称可能随 USB 插拔改变。设备身份集中配置在 [`configs/hardware/can.yaml`](../configs/hardware/can.yaml)，启动时再解析成当前接口名：

```yaml
adapters:
  left_leader: {usb_serial: '004B001F4648571220363032'}
  left_follower: {usb_serial: '0033001E4648570920363032'}
  right_leader: {usb_serial: '003400164648570220393831'}
  right_follower: {usb_serial: '0039004A4648571220363032'}
```

模型配置通过 `robot.can_config: hardware/can.yaml` 指定此文件，`pairs` 中只需填写设备开关：

```yaml
robot:
  arms: [left, right]
  pairs:
    left:
      leader: false
      follower: true
    right:
      leader: false
      follower: true
```

设为 `true` 时，配置加载器自动用 `<侧别>_<角色>` 查找适配器，例如 `left.follower` 对应 `left_follower`；设为 `false` 不读取该设备映射，也不连接。需要人工接管时将所选侧的 `leader` 改成 `true` 即可。`arms` 同时决定模型维度；要只运行一侧，应修改 `arms`，而不是将所选侧的 `follower` 关闭。

这些序列号对应当前工作区的适配器，更换适配器时更新此文件即可。唯一 USB 序列号不依赖插入的 USB 端口，因此同一个适配器换插口后仍能找到；适配器到机械臂的 CAN 接线必须保持原来的对应关系。序列号标识的是适配器，不是机械臂。

无可用唯一序列号时，可将该项改为 `usb_path: '1-5.3'`，绑定固定物理 USB 口；也支持 `1-5.3:1.0` 形式。使用 `usb_path` 后需保持 USB 插口不变。

```bash
python scripts/prepare_can.py --list
python scripts/prepare_can.py --config configs/act_left.yaml
python scripts/prepare_can.py --config configs/act_left.yaml --execute
```

`--list` 不需要配置文件，只读取并列出当前 CAN 接口及 USB 标识。`--config` 解析所选设备的当前接口并打印 `down → bitrate → up` 计划，不修改网卡；加 `--execute` 才执行这些配置命令。默认波特率为 1 Mbps。遥操或采集时应传入对应配置，以包含需要的主臂。

配置加载本身不扫描实机，允许无硬件检查配置与运行模拟器。真实 SDK 连接和 CAN 准备时才通过本机接口信息解析 USB 标识；缺失或匹配到多个接口会明确报错，不回退到旧 `canN`。这个解析只确定设备地址，不额外发送机械臂状态或控制命令；项目不安装 udev 规则，也不重命名接口。

不支持运行中拔插后的自动重连。重插后重新执行 CAN 准备命令并重启工作流，即会重新解析编号。

## 手动失能、恢复、复位与回放

程序退出仍执行电子停止并断开，不自动失能。需要单独释放关节与夹爪时，先退出控制进程，并放稳或托住机械臂及夹持物，再运行：

```bash
python scripts/disable.py --config configs/collect.yaml
python scripts/disable.py --config configs/collect.yaml --execute
```

不带 `--execute` 仅预览所选设备。设备范围由 `robot.arms` 和主从臂开关决定；上述 `collect.yaml` 包含左右四臂。脚本通过 SDK 连接，分别调用 `arm.disable()`、`gripper.disable_gripper()`，然后断开。每条命令只调用一次；某条命令抛出异常后仍尝试其余设备，最终返回失败。SDK 的即时返回值来自缓存，脚本不将其当作硬件确认，也不额外轮询反馈。此操作不改变正常工作流的退出行为。

`scripts/recover.py` 的 `--execute` 直接对所选机械臂调用 `reset()`，然后断开。该命令不使能、不移动，也不轮询状态。

`scripts/reset.py` 将 `robot.home_position` 作为一次目标提交。`configs/home.yaml` 保留第 5 关节 24° 和夹爪 0.1 m 的显式目标。`scripts/replay.py` 按轨迹相对时间逐条提交，格式如下：

```yaml
action_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4,
               left_joint_5, left_joint_6, left_gripper_width]
action_semantics: absolute_joint_position_rad_gripper_m
timestamps: [0.0, 1.0]
actions:
  - [0.0, 0.0, 0.0, 0.0, 0.4188790204786391, 0.0, 0.1]
  - [0.0, 0.0, 0.0, 0.0, 0.4188790204786391, 0.0, 0.08]
```

```bash
python scripts/reset.py --config configs/home.yaml
python scripts/replay.py --config configs/act_left.yaml --trajectory trajectory.yaml
```

默认只打印计划；`--mock` 使用模拟器，`--execute` 使用实机。最后一条目标发送后固定等待 `--hold-seconds`，默认 2 秒，然后停止并断开。回放不插值、不自动接近首个点、不检查到位，等待时长由操作者指定。

## 参考与版本

实现参考 [官方 Piper 示例](https://github.com/agilexrobotics/PyAgxArm_demo/tree/d72c390588c76df811b59dc0c9ca85a29fca3251/piper)、[主从人在环教程](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/vq476s29h82fbdrf) 和 [核对的 pyAgxArm 源码](https://github.com/agilexrobotics/pyAgxArm/tree/d90e234ba8e408cfa964b38aa9120b3131fc9f85)。只调用公开 SDK 接口，已去掉对 `_parser` 和底层 `send_bus` 的依赖。

精简前的框架保存为 `.legacy/guarded-framework-20261023-124249.tar.gz`，用于代码对照；当前运行路径没有额外的安全模式开关。
