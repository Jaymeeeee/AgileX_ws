# 工作空间验证记录

[返回工作空间 README](../README.md)

本页集中保存历史验证结果，按记录时的代码、配置和设备条件理解。软件测试、短时录包和固件查询分别说明各自验证范围，不等同于实机运动、停止效果或任务成功率验收。

## 完整采集录包摘要

双组机械臂和三相机曾完成约 20 秒实机录制，27 个预期话题均有数据，并通过逐条读取检查。原记录引用的结果文件为 `bags/test.validation.json`，原始文件为 `bags/test.bag`；当前工作空间未找到这两个文件，此处保留历史路径供追溯。这属于短时功能验证，长期稳定性和训练数据时序仍需单独评估。

## ACT 部署、固件查询与复位（2026-10-16）

2026-10-16 验证记录：14 项部署单元测试、14 项硬件测试、10 项录包测试通过；私有 ROS master 的四类集成测试通过。4090 离线推理中位数约 8 ms。真实三相机和双臂话题下 dry-run 10 秒完成 150 次推理，进程通信加推理往返 P95 约 22 ms，全程 CAN 保持只接收。当时实机摆放触发目标偏差检查（例如右侧 J6 预测与实测偏差约 0.76 rad、右夹爪约 100 mm）；需准备训练时的起始姿态和场景后重新检查，不能通过放宽阈值来代替定位。未执行 SDK 使能/动作或实机保持测试。

固件查询补充（2026-10-16）：两侧实机均读到 `S-V1.9-0`，SDK 自动选择 `v189`，已写入 `expected_firmware`。连接后先等待新鲜 SDK 关节反馈及接收频率稳定，再查询固件；无完整响应最多尝试 3 次。此处理避免刚连接时 SDK 接收频率为零导致响应被清空的时序风险，失败仍禁止使能和运动。新增回归测试后部署单元测试共 17 项通过。本次仅验证查询，不代表运动/保持已实机验收。

复位补充（2026-10-16）：新增 10 项模拟测试，覆盖反馈确认的动作顺序、受阻超时、取消、数据失效、起始快照保存和控制互斥。当时部署/复位 30 项、硬件 14 项、录包 10 项测试及 catkin 编译通过；旧日志回位计划已用 `--dry-run` 检查，尚未执行实机复位。

## 硬件与录包验证背景（2026-10-14）

以下硬件与录包记录日期为 2026-10-14（Asia/Shanghai）；其中“当前”均指该次记录时的版本。

### 结构化 MotorFeedback

`/piper/{side}_bus/motor_feedback` 改用 `hardware_bringup/MotorFeedback`，保留 header、
joint_order、current_a、motor_position_rad、motor_velocity_rad_s、torque_estimate_nm。
原始字节、六分片时间、型号和系数仍通过 events 保存，数组数值不舍入。

- 消息生成及构建通过，21 项单元测试通过，包含 ROS 序列化往返检查。
- 两侧真实 ROS 连续订阅 15 秒：约 100.04 / 100.07 Hz，1503 / 1502 条消息。
- 两侧各匹配 200 条结构化消息与同时间戳的 events，五个数组逐元素完全一致。
- 九路相机发布及从臂监控保持正常；主臂目标在节点重启后需重新实际观测。
- 隔离模拟录包和故障测试通过；测试 bag 两侧各 48 条结构化消息均可正确反序列化。
- `rostopic echo` 完整消息和单独 torque_estimate_nm 字段均已实测。

记录：`logs/structured_motor_build.log`、`logs/structured_motor_tests.log`、
`logs/structured_motor_ros_validation.json`、`logs/structured_motor_recording.log`、
`logs/structured_motor_bag_readback.json`。旧 bag 保持旧 String 类型，不自动改写。

### 主臂跟随观测与六关节力矩估计

用户确认：左右均为标准 Piper，固件版本未知，末端没有六维力传感器，电脑接在主从
互连总线上，手动操作左主臂时左从臂可跟随。

- 初始 5 秒全 ID 采样：两侧均无 0x155/156/157/159；0x251…256 每个 ID 均收到 500 帧。
- 随后 45 秒采样包含用户确认的左侧跟随操作：can0 四个目标 ID 各收到 4,531 帧，
  原有 ROS 桥接也实际解码并发布 4,531 条关节目标和夹爪目标。
  操作结束后约 60 秒仍保持原始目标时间戳，没有伪造更新；右侧本轮未操作、未见目标。
- 因此左侧主臂目标在跟随操作期间可见；此前未收到不等于主臂或解码失败。
  没有发送模式配置或控制指令，未确定具体触发广播的硬件按键/动作机制。
- 新增每侧 /piper/{side}_bus/motor_feedback，保留六个电机电流、SDK 电机轴状态、
  原始帧和各自时间戳，按用户确认的标准 Piper 型号换算 torque_estimate_nm。
  使用安装的 pyAgxArm k*b*c 系数（J1–3:4.725；J4–6:0.7787701152 N·m/A）。
- 两侧同时实测 15 秒：完整六电机 ROS 向量约 96–98 Hz；逐项验证
  torque_estimate_nm == current_a * torque_coefficients_nm_per_a，六帧跨度符合配置。
  这是电流推算的驱动力矩，未做载荷标定，不是关节力矩传感器读数或末端外力。
- 未获取或伪造末端六维力/力矩；现有设备无六维力传感器，未实现动力学外力估计。
- 21 项单元测试通过：包括正负电流、型号系数差异、未知型号不伪造 N·m、缺帧/超时。
- 隔离 ROS master 录包测试通过，新必需清单 23 个话题，含两侧电机反馈；故障注入仍
  标记 incomplete。真实主臂目标是否齐全独立检查。
- 三台相机仍严格只有九个图像话题。固件查询方法已记录，尚未实际发送查询请求。
- 最终用户操作左右两组及夹爪后，当前会话左关节/夹爪目标各 4,111 条，右侧各
  3,527 条，四路全部通过，硬件状态变为 ready。目标停止广播后保留其原始时间戳，
  不等于主臂持续心跳或独立实测反馈。

记录：`logs/leader_motor_probe_20261014_204322.json`、
`logs/leader_following_probe_20261014_204918.json`、`logs/leader_ros_observed.json`、
`logs/piper_torque_ros_validation.json`、`logs/motor_feedback_tests.log`、
`logs/piper_torque_recording_integration.log`、`logs/piper_torque_live.log`、
`logs/piper_torque_final_status.json`。

### 每台相机仅三个话题

20:38 后切换到 `hardware_bringup/realsense_images`，直接使用当前 ROS 环境的
librealsense2 2.50.0（与此前官方 ROS 驱动使用的版本相同）。
只发布 color/image_raw、depth/image_rect_raw、aligned_depth_to_color/image_raw。
不再发布 camera_info、压缩传输、metadata、参数更新、bond 或相机 TF。
内参与 depth-to-color 外参保留在 /cam_*/calibration 参数中，硬件状态和录制 manifest
也包含该数据。

- 构建通过，17 项单元测试通过。
- 同时订阅真实三相机与两侧 Piper，连续 25 秒验证通过。
- ROS master 上相机发布话题集合严格等于要求的九个话题。
- 九路图像均 640×480、约 14.989 Hz，每路采到 375 帧；图像内容、尺寸、编码有效。
- 图像源时间戳严格递增，最大间隔约 66.72 ms；原始深度与对齐深度时间戳对应。
- 左右从臂关节、夹爪、状态约 200 Hz；四个主臂目标话题仍缺失，系统保持 degraded。
- 独立 ROS master 上的模拟录包测试通过；新必需清单共 21 个话题（12 个机械臂话题、
  9 个相机话题），故障注入后标记 incomplete。模拟结果不代表缺失主臂目标已实测。

记录：`logs/three_topic_build.log`、`logs/three_topic_tests.log`、
`logs/three_topic_live.log`、`logs/three_topic_ros_validation.json`、
`logs/three_topic_recording_integration.log`。
以下包含精简前的历史验证，涉及 camera_info 的数量以当时版本为准。

### 早期已完成项目

- 三个 Catkin 包编译通过，使用系统 Python 3.8 和 ROS Noetic。
- 单元测试覆盖相机序列号/命名绑定、CAN 模式检查、完整关节分片组装、单位、
  过时/乱序数据、夹爪模式、故障状态、图像长度、录制完整性要求和配置变化。
- 真实 cam_l 独立启动、数据检查、rosbag 录制和正常收尾通过。
- 真实三相机 cam_h/cam_l/cam_r 逐台启动，640×480、15 FPS，十八个图像/内参话题正常。
  所有相机就绪后又连续监测 20 秒；最后观察帧率 14.98–14.99 Hz。
- 三相机录包经过完整逐消息读取；九路图像约 14.99 Hz，分别 189–190 帧，
  未见消息序号间隙或时间戳倒退，像素数据长度正确。测试 bag 约 461.4 MiB。
- 隔离 ROS master 上用模拟两组主从臂与三相机话题完成完整录包，
  必需 30 个话题都有记录；静止的主臂目标以原始时间戳写入 bag；
  注入硬件故障后录制自动停止并标为 incomplete。

### 接线修复与包改名后的实机复测

早期左 can0 无报文；用户修复断线并重新启用后，两接口均为 1 Mbps、UP、LISTEN-ONLY。
20:15 的 10 秒采样：左 24,227 帧、右 24,218 帧，均无新增接收错误或丢包；
两侧从臂关节、夹爪、状态均约 200 Hz，从臂 arm_status=0、err_code=0。

包已改名为 `hardware_bringup`、`data_collection`、`model_deploy`，Python 模块、
依赖、launch、入口脚本和文档引用同步更新，清理旧构建目录后重新编译成功。

通过 `hardware_bringup` 同时启动两侧状态桥接和三台相机，在独立 ROS 订阅节点上
连续采样 20 秒：

- 两侧从臂各四个关节/夹爪/状态话题通过内容、时间戳和新鲜度检查，约 199.58–200.10 Hz。
- 三相机十八个图像/内参话题全部通过，640×480、14.98–15.00 Hz。
- 九路图像编码、尺寸与数据长度有效；抽样彩色图有像素变化，深度图包含非零值。
  对齐深度抽样非零像素比例约 86%–97%；该比例不等于深度精度验证。
- 实际执行 `rostopic echo` 成功读取左从臂六关节、右从臂夹爪、第三视角深度消息头。
- 两侧四个主臂目标/夹爪目标话题均未收到消息；系统保持 `degraded` 并继续发布已有数据。
  完整录制仍不放行。`require_leader_targets:=true` 可恢复严格的启动检查。

全程未发送运动、使能、回零或夹爪指令，也未补齐或伪造缺失目标。

### 原始记录

- `logs/build.log`、`logs/unit_tests.log`
- `logs/can_probe.json`
- `logs/cam_l_bringup.log`、`logs/validation_bags/`
- `logs/three_cameras_bringup.log`、`logs/three_cameras_ready.json`
- `logs/three_camera_bag_check.json`
- `logs/three_camera_bags/20261014_195502_cb575d2d/manifest.json`
- `logs/synthetic_validation/result.json`、`logs/integration_smoke.log`
- `logs/can_diagnostic_20261014_201556.json`
- `logs/package_rename_build.log`、`logs/package_rename_tests.log`
- `logs/hardware_renamed_live.log`、`logs/hardware_renamed_ros_validation.json`

上述早期测试的真实 bag 与模拟 bag 分目录保存，均未放入用户正式采集目录。完整采集录包摘要所引用的历史文件路径另见上文。

### 验证范围与限制

相机启动阶段曾出现 USB control_transfer、Depth stream start failure 和 Left MIPI error 通知，
后续数据流恢复并通过上述检查。管理器以实际有效数据判断就绪；这些 SDK 通知仍保留于日志。
短时测试不保证长期运行、所有 USB 连接方式或更高分辨率/帧率稳定。
该日未做绝对深度精度、跨相机硬件同步、相机与机械臂外参标定、力矩测量验证或模型部署。后续部署测试另见本页 2026-10-16 记录。
