# 配置契约

配置由 `robot`、`policy`、`observation`、`workflow`、`record` 五部分组成。使用 `extends` 继承；重复 YAML 键、继承循环、非有限数值和不一致的动作定义会报错。`camera_map` 整体替换，不会在单臂配置中残留另一侧相机。

```yaml
extends: base.yaml
robot:
  arms: [left]                     # [right] 或 [left, right]
policy:
  backend: lerobot
  type: act                        # 显式选择模型，禁止 auto
  checkpoint: /models/left/pretrained_model
  python: /envs/lerobot/bin/python
  lerobot_source: /sources/lerobot   # 可为 null，使用已安装版本
  device: cuda
  execution: auto                  # auto / step / chunk
observation:
  task: 用左臂完成插接
  camera_map:
    observation.images.cam_h: cam_h
    observation.images.cam_l: cam_l
  image_resize: none
workflow:
  mode: deploy
  learner: none
  policy_hz: 15
  auto_start: false
record:
  backend: none                    # none / files / ros
  output_dir: ../outputs/sessions/piper
  python: /usr/bin/python3          # 仅 ros 后端使用
  ros_setup: /opt/ros/noetic/setup.bash
```

`robot.can_config`、`checkpoint`、`lerobot_source`、带路径的解释器、输出目录、ROS setup 以及 `workflow.rlt` 中的路径相对于声明它们的 YAML 文件解析。支持 `~` 和环境变量。模型 `overrides` 保留其原字段语义，本地模型或分类器路径请写绝对路径。

## 硬件与观测

`robot.arms` 明确选择左、右或左右两侧，`pairs` 中的 `leader`、`follower` 用 `true` / `false` 决定是否连接对应设备。普通部署示例默认 `leader: false`、`follower: true`，因此单臂部署只连接一台从臂，双臂部署只连接两台从臂。模型状态和动作都按从臂组织，每侧 6 个关节加夹爪宽度，共 7 维。双臂按左、右顺序共 14 维。

`configs/base.yaml` 用下面的方式引用独立 CAN 配置：

```yaml
robot:
  can_config: hardware/can.yaml
  arms: [left, right]
  pairs:
    left:
      leader: false
      follower: true
    right:
      leader: false
      follower: true
```

`can_config` 文件的顶层是 `adapters`，名称映射到 USB 标识，例如：

```yaml
adapters:
  left_follower: {usb_serial: '0033001E4648570920363032'}
  left_leader: {usb_serial: '004B001F4648571220363032'}
```

`true` 自动查找 `adapters` 中的 `<侧别>_<角色>`：例如 `left.follower: true` 使用 `left_follower`，`right.leader: true` 使用 `right_leader`。`false` 不读取该设备的映射，也不连接设备，无需重复填写适配器名称。USB 标识只在这个文件维护。`usb_serial` 唯一时，换 USB 插口或 `canN` 编号变化都无需改配置；没有唯一序列号可用 `usb_path: '1-5.3'` 绑定固定 USB 口，也接受 `1-5.3:1.0` 形式。完整示例及查看当前 USB 标识的命令见 [CAN 配置](hardware.md#can-配置)。

只有 `robot.arms` 选中的侧参与连接；例如 `arms: [left]` 时，即使继承的 `right.follower` 为 `true`，也不会连接右臂。每个所选侧必须保持 `follower: true`，设为 `false` 会报配置错误；要去掉某侧，应从 `arms` 中移除该侧，以同步调整模型状态与动作维度。

`load_config()` 会把开关展开为内部设备配置并读取相应 USB 标识，不扫描或连接硬件；SDK 连接和 CAN 准备时才把 USB 标识解析为当前接口名，缺失或有歧义直接报错。使用序列号固定的是适配器身份，适配器与机械臂的接线关系仍应保持一致；运行中拔插后需重新准备接口并重启。

需要人工接管时，为每个所选侧补充主臂。以下例子为单左臂部署增加人工接管：

```yaml
extends: act_left.yaml
robot:
  pairs:
    left:
      leader: true
```

`teleop.yaml`、`collect.yaml`、`rlt_online.yaml` 和 `mock.yaml` 已为所选侧设置 `leader: true`。继承带主臂的配置后，如需只用从臂，显式设置 `leader: false`；仅省略该字段会继续继承已有值。没有主臂时不创建转发器，不能切换到人工接管。

规范名称为 `left_joint_1` 至 `left_joint_6`、`left_gripper_width`，右侧同理。关节用 rad，夹爪宽度用 m。状态来自从臂反馈；动作是绝对关节位置和夹爪宽度。`observation.state_names` 与 `action_names` 可声明模型训练时的排列顺序，但必须各自完整覆盖所选侧，不能靠裁剪维数换用另一种机器人配置。

机械臂命令参数为 `move_speed_percent`（直接传给 SDK，默认 100）、`gripper_force` 和 `mode_settle_sec`（模式切换后的固定暂停，默认 0.1 秒）。配置中不再设置软件限位、每步速度限制、对齐容差、反馈年龄或跟踪误差阈值。固件字段只选择 SDK 协议配置，不读取硬件版本进行比对。

`robot.can_tx_queue_len` 是 SocketCAN 接口的发送队列长度，默认 64 帧，由 `scripts/prepare_can.py --execute` 应用到所选 USB-CAN 接口；修改 YAML 后需重新执行该准备命令，运行中的程序不会自行修改系统网络接口。这个队列用于吸收短时突发；扩大队列也可能增加积压延迟，不用于掩盖持续通信故障。主从转发对 `ENOBUFS/EAGAIN` 最多重试 10 ms，其它发送错误立即上报，持续拥堵仍终止转发。

`robot.startup_enable_timeout_sec` 控制初始反馈等待、复位确认和启动使能各自的最长等待时间，默认各 10 秒。自动启动先检查已有驱动报警，再将所选主臂切回 follower 模式，使上次退出后保留 leader 模式的主臂恢复普通反馈；随后检查所有所选设备的机械臂状态与六关节驱动反馈；存在驱动报警则退出，仅对电子停止的设备复位。SDK 的 `reset()` 会令关节失能，对这些设备等待六关节反馈均晚于复位发令且全部失能，再进入使能阶段。SDK 的 `enable()` 只发送命令并返回缓存状态，框架每 20 ms 对尚未完成且没有驱动报警的设备重试，确认所有所选臂的六个关节反馈均晚于首次使能发令且全部已使能后才提交第一个位置目标。缺失、发令前的缓存或未完成状态都会在超时中报告。

纯人工 `collect/teleop` 配合 `workflow.auto_start: true` 会按需恢复电子停止并完成使能，与模型部署共用一次 SDK `move_js()` 初始目标发送：读取从臂当前姿态，向主从臂各发一次关节及夹爪目标。随后只观察新反馈，全部所选主从臂在 0.8 秒窗口内，每个关节的位置范围不超过 0.002 rad、夹爪不超过 0.001 m，即视为运动稳定；同一缓存时间戳不会推进窗口。不重复发送目标，也不要求主从精确对齐。`pair_errors` 和 `target_errors` 分别记录主从当前姿态差及相对初始目标的偏差，均只用于诊断。

`robot.startup_alignment_timeout_sec` 控制初始目标提交之后的运动稳定等待时间，默认 10 秒；运动未稳定或缺少可用反馈时会超时停止，并报告主从差以及各臂目标、实际位置、关节／夹爪误差。期间发现驱动故障或使能丢失会停止，不自动再使能或清错，停止指令也会中止等待。这段等待只用于纯人工自动启动；稳定后调用与模型部署相同的人工接管入口。

`camera_map` 左侧是 checkpoint 输入名称，右侧是硬件相机名称。相机由显式序列号绑定。双臂采集 `collect.yaml` 默认选择三台相机；右臂单臂采集 `collect_right.yaml` 将 `camera_map` 替换为顶部 `cam_h` 和右腕 `cam_r`，录包清单中的 `{camera}` 随之只展开这两台。观测中的 RGB 为 HWC uint8，模型适配层转换为其所需张量；默认不在采集层额外缩放。`image_resize: checkpoint` 是显式选择，须与训练图像处理一致。

`observation.depth: true` 启用原始深度；同时设置 `observation.align_depth: true`，会调用 RealSense SDK 将深度对齐到 RGB，并保留原始深度。基础部署配置默认关闭这两个开关，`collect.yaml` 默认同时启用，`collect_right.yaml` 继承；也可在 `observation.cameras.<名称>` 中分别覆盖。开启 ROS 采集后，每台相机发布 `/piper/cameras/<名称>/{color,depth,aligned_depth_to_color}/image_raw` 及对应的 `camera_info`，只发布已启用的流。RGB 编码为 `rgb8`；两种深度编码均为 `32FC1`，单位为米，无效零值转换为 NaN。对齐深度使用 RGB 的尺寸、内参和光学坐标系，时间戳保留 SDK 对齐帧的采集时间。文件采集中的两种深度均保存为原始 `uint16` NPY，并记录 `depth_scale_m`。

模型输入校验同时检查状态、动作、相机名称及可选的 checkpoint `deployment.json`。有额外传感器输入却没有硬件映射的模型会报错。预处理、后处理、归一化和内部填充沿用保存的处理器。

## 工作流与采集彼此独立

`deploy` 加载模型，主臂可选；`teleop` 和 `collect` 只进行 SDK 主从控制，要求所选侧都设置 `leader: true`；`online_rl` 加载模型和 learner，示例默认带主臂，也允许设置 `leader: false` 做无人工介入的在线学习。`p/h/hold` 管理控制权，其中 `h` 要求所选侧的主臂都存在；缺少主臂时拒绝接管请求并继续模型运行。`s/f/t` 管理回合，不通过 ROS 消息改变机械臂状态。

`record.backend` 控制采集：

| 值 | 行为 |
| --- | --- |
| `none` | 不启动采集，不创建数据目录，也不创建 ROS 进程 |
| `files` | 保存观测图像、实际动作、事件与 manifest，无 ROS |
| `ros` | 保存以上文件，并通过独立 ROS 子进程发布标准消息和写 rosbag |
| `ros_publish` | 仅通过 ROS 子进程发布消息，不创建文件；用独立 `scripts/record.py` 手动录 bag |

`collect` 支持自动保存或 `ros_publish` 后单独录制；`online_rl` 必须选能保存轨迹的 `files` / `ros`，不支持仅发布。纯部署和纯遥操默认 `none`，在线 RLT 默认 `files`；需要同步录包时显式改成 `ros`。ROS 子进程通过 `record.python` 启动，仅它加载 `record.ros_setup`。已有自包含 ROS 环境可将 setup 设置为 `null`。

`configs/collect_right.yaml` 使用 `ros_publish` 和 `auto_start: true`，启动后自动对齐、进入主从控制并发布消息；另行执行 `python scripts/record.py --config configs/collect_right.yaml --output bags/right_demo_0001` 才创建并录制该段 `data.bag`。录制终端 Ctrl+C 关闭 bag，不影响控制进程。输出目录由 `--output` 指定，`record.output_dir` 不用于这个模式。`h/hold/s/f` 仍只改变控制或回合状态，不开始或停止独立录制。纯人工 `collect/teleop` 配合 `ros_publish` 时可设置 `workflow.max_episode_sec: null`，使人工控制持续到手动暂停或退出；模型部署和在线学习仍需有限回合时长。

`record.topics_config` 指定独立的录制清单，路径相对于声明它的配置文件解析。`collect.yaml` 指向 [`configs/recording/topics.yaml`](../configs/recording/topics.yaml)，右侧采集继承同一清单。文件包含两个字符串列表：`required` 中每个 topic 都必须至少收到一条消息，`optional` 中的 topic 有消息时保存、没有时不影响完整性。支持 `{prefix}`、`{side}`、`{camera}` 模板；侧和相机按当前配置展开，主臂项仅对已启用主臂的侧展开。每项最终精确匹配一个 topic。清单不隐式开启或关闭相机流；例如要求 `depth/image_raw` 却关闭 `observation.depth`，最终会报告缺失。

默认必需数据如下；`<侧>` 是所选机械臂，`<相机>` 是 `camera_map` 中实际使用的相机。表中 `/piper` 可由 `record.topic_prefix` 替换。

| topic | 内容 |
| --- | --- |
| `/piper/state` | 所选从臂合并后的关节与夹爪实测状态 |
| `/piper/action`、`/piper/action_metadata` | 已提交的模型目标或实际转发的人工目标，以及来源和回合信息 |
| `/piper/arms/<侧>/follower/joint_states`、`diagnostics` | 从臂 SDK 状态与诊断 |
| `/piper/arms/<侧>/leader/command` | 主臂输出的关节与夹爪控制数据 |
| `/piper/cameras/<相机>/{color,depth,aligned_depth_to_color}/image_raw` | RGB、原始深度、对齐到 RGB 的深度 |
| 同一相机流的 `camera_info` | 对应图像的标定信息 |

`events`、主臂 `joint_states` 和主臂 `diagnostics` 默认是可选项。主臂切为 leader 模式后通常只发送控制帧，不持续发送状态反馈；`leader/command` 不能标成新的实测关节状态，保留的诊断也必须结合其原始时间戳理解。

```bash
python scripts/record.py --config configs/collect_right.yaml --list-topics
python scripts/record.py --config configs/collect_right.yaml \
  --topics-config configs/recording/my_topics.yaml --output bags/right_demo_0001
```

`--list-topics` 只预览，不连接硬件或创建 bag；`--topics-config` 覆盖本次手动录包的清单。录制结束后，`manifest.json` 保存展开后的必需／可选列表、实际消息数量和 `missing_required_topics`。缺少任一必需 topic 时状态为 `incomplete`，脚本非零退出，bag 仍保留；写入或进程错误为 `failed`。这项检查只判断本段录制是否完整，不改变机械臂使能、对齐或控制状态。未设置 `topics_config` 的旧配置仍按机械臂、相机和前缀选择 topic，但没有该显式必需项契约。

`robot.control_hz` 是状态读取与操作者命令处理频率，不得低于 `workflow.policy_hz`。模型目标按 `policy_hz` 原值提交；下一次推理尚未完成时重复最后提交目标。状态和图像使用 SDK 最新缓存，保留原始时间戳，不以年龄拒绝样本。

仍保留模型名称／维度／单位校验、子进程请求超时和采集写入错误处理；这些用于让模型加载、进程通信和数据保存的错误明确报出。退出及显式 `stop` 仍调用 SDK 停止。

## 模型与 learner

`policy.type` 选择 LeRobot 注册模型，再加载匹配 checkpoint。两者类型不同会在使能前失败。需要历史观测或时间集成的模型走逐步接口；具备明确能力契约的模型可走 chunk 接口。RTC 和无法转换成当前绝对关节动作定义的模型不会被隐式兼容。

普通 LeRobot 模型可以作为部署策略，但 RLT learner 当前只支持自定义 `pi05_rlt`。使用 `workflow.mode: online_rl` 时还必须明确 `workflow.learner: pi05_rlt`。模型种类、机器人配置和是否采集属于不同配置项。
