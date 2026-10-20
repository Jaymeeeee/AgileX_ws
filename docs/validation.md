# 工作空间验证记录

[返回工作空间 README](../README.md)

本页集中保存历史验证结果，按记录时的代码、配置和设备条件理解。软件测试、短时录包和固件查询分别说明各自验证范围，不等同于实机运动、停止效果或任务成功率验收。

## 四路独立 CAN 主从控制（2026-10-19）

通过逐台拔出 USB 适配器辨认四臂，映射及启动方法见[主从控制](teleop.md)。
四臂均查询到 `S-V1.9-0`；配置初始化、只读 ROS 检查、当前位置使能保持和主臂自动对齐已实测。
用户确认左右关节、夹爪对应正确；将跟随调整为 100 Hz、SDK 100%、不额外逐步限速后，
用户确认速度明显改善、无明显滞后。自动对齐仍限速，ACT 部署原有速度参数保持不变。

- 45 秒实机运动观测：左右人工组合目标、SDK 下发目标、实际反馈各约 100 Hz，
  两侧夹爪接管完成，遥操作状态无故障。频率不等于端到端延迟测量。
- 不带相机短录制约 21.9 秒，manifest 为 complete；完整读取 83,069 条消息，
  26 个预期话题均有数据。静止的原生主臂目标按原始时间戳锁存，未伪造更新。
- catkin 构建通过；完整回归通过：硬件 15 项、采集 10 项、部署与遥操作 97 项，共 122 项。
- 结束录制后暂停跟随、退出全部本次控制节点；从臂请求保持，示教主臂请求阻尼停止。

记录：`logs/teleop/fast_motion_validation_20261019_201859.json`、
`logs/teleop/recording_readback_20261019.json`、`logs/teleop/four_can_tests_20261019.log`、
`logs/teleop/four_can_build_20261019.log`、`bags/teleop_validation_20261019_four_can/`。
本次未重测三相机联合录制、模型部署或模型/人工控制仲裁，未实现 RLT。

### 模式区分与采集复位补充（2026-10-19）

README 将操作区分为数据采集、一般模型部署、RL 人在环；按用户确认，本次只完善前两者，
RL 人在环及其主臂同步、人工接管、四臂复位留待后续实现。
普通部署的 CAN 准备默认只访问从臂 can3/can0，控制器/独立复位保持仅连接从臂。
新增回归验证即使配置中存在两台主臂，部署也不检查或连接它们。

采集增加 `r` 后按回车：先保存当前段并暂停跟随，保持主臂，从臂依次完成原有复位流程，
再把两台主臂对齐到从臂并保持。固定目标共用 `act.yaml` 的 `reset.home_joints_rad`。
复位通过节点已有 SDK 执行，不创建额外 CAN 控制器；完成后输入 `d` 才恢复跟随。

- 用户确认现场准备就绪后，完成两轮四臂复位，节点无故障；第一轮左右从臂实际抬升约
  100.09 / 100.12 mm，最终从臂关节相对固定目标的最大偏差约 0.03°，主臂约 0.80°。
- 第一轮后 `d` 恢复主从跟随和夹爪接管，硬件 READY；第二轮在跟随和录制状态下按 `r`，
  先保存 9.17 秒录制，再退出示教、复位四臂；录制 CLI 正常退出，释放下一段录制所需的锁。
- 录制 manifest 为 complete，完整读回 26 个话题、32,503 条消息，无缺失话题。
- 构建、脚本语法检查通过；完整回归 133 项通过（硬件 15、采集 11、部署/遥操作 107）。
- 两轮测试结束后已退出采集控制节点，用户现场确认复位正常。

普通部署检查：用户关闭两台主臂电源后，CAN 准备只访问 can3/can0，两台从臂反馈正常，
三台相机曾全部 READY；左右从臂固件查询均为 `S-V1.9-0` / SDK profile `v189`。
首次检查右相机断流，重启后报 `set_xu(ctrl=1) failed! Last Error: Resource temporarily unavailable`；
随后 SDK 只枚举到 cam_h、cam_l。用户重插 cam_r 后三台相机恢复枚举，ACT 只推理运行
107 帧、约 7.08 秒，推理中位数约 8.77 ms，无动作检查错误、无实际下发目标。
之后 cam_r 再次断流，只推理以 `cam_r data stale` 停止，硬件管理器随后也停止。
因此仅从臂的通信及模型输入/推理链路已验证，完整 10 秒稳定性检查未通过；
右相机持续采集问题待处理。本次没有执行普通模型运动或普通部署的独立实机复位。
所有本次启动的控制、硬件及推理进程均已退出。

记录：`logs/teleop/collection_reset_validation_20261019.json`、
`logs/teleop/collection_reset_recording_20261019.json`、
`logs/teleop/modes_reset_tests_20261019.log`、`logs/teleop/modes_reset_build_20261019.log`、
`logs/teleop/20261019_215823/`、`bags/collection_reset_validation_20261019/`。
普通部署记录：`logs/deploy/follower_only_validation_20261019/validation.json`、
`logs/deploy/follower_only_validation_replug_20261019/hardware_ready.json`、
`logs/deploy/20261019_220352_220144/firmware.json`、
`logs/deploy/20261019_220948_177170/`（107 帧只推理及断流结果）。

## 右相机换 USB 插口后复测（2026-10-19）

用户将右相机从原拓展坞换到另一插口后，SDK 识别 `cam_r` 序列号仍为
`262522070143`，物理端口 `2-2-73`；USB 拓扑为主控制器直接连接，链路 5000 Mb/s，
SDK USB 类型为 3.2。三台相机均可枚举，序列号配置无需修改。

- 右相机单独持续采集 60 秒，RGB、深度、对齐深度各 900–901 帧，约 14.995 Hz。
- 三相机与两台从臂只读 CAN 反馈联合运行 120 秒，九路图像各 1798–1799 帧，
  约 14.988–14.994 Hz；均为 640×480，图像编码及像素数据长度正确。
- 两阶段均无断流、时间戳倒退或图像校验错误。联合测试九路图像最大源时间间隔
  约 67.05 ms，最大订阅回调到达间隔约 83.70 ms。
- 联合测试期间 ACT 只推理完整运行 30 秒、450 帧，推理耗时中位数约 10.54 ms，
  无动作检查错误，实际下发目标为零；此前受右相机中断的普通部署只推理复测通过。
- 保持原 640×480、15 FPS 配置和既有数据时效检查；没有修改驱动、放宽检查或执行实机运动。
  测试结束后已退出本次硬件与推理进程。

本次换口后未复现此前故障，说明当前 USB 连接下短时运行稳定；尚不能仅凭换口测试
区分原拓展坞带宽、供电或连接稳定性等具体原因，也不等于长期稳定性验收。
记录及复测脚本：`logs/camera_retest_20261019/`，其中
`right_only/result.json`、`three_cameras_can_inference/result.json` 保存逐话题统计，
`device_info_before.txt`、`usb_topology_before.txt`、`usb_topology_after.txt` 保存连接信息；
模型运行日志为 `logs/deploy/20261019_221755_637015/`。

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

## 单臂选择的软件验证（2026-10-20）

- `test.sh`：硬件模块 15 项、录制模块 12 项、部署/遥操作/复位模块 117 项，全部通过。使用模拟设备；IPC 测试在允许本地 socket 的环境执行。
- 转换测试 10 项通过，包含左右两侧“bag 完全没有另一侧话题”的实际 rosbag → LeRobot 7 维数据集写入、读回及名称/数值校验。总计 154 项。
- 左/右单臂分别使用随机小 ACT 网络验证：7 维配置 → 权重/处理器保存 → 推理进程重新加载 → 三相机输入 → 有限 7 维输出。只是流程测试，不能作为可部署的任务权重。
- `roslaunch --nodes` 展开结果：单臂为硬件管理器＋该侧两个主从节点；默认双臂为硬件管理器＋四节点。没有启动这些设备节点。
- 左、右、默认双臂的 `reset_node.py --dry-run` 均通过，固定姿态、维度和 CAN 映射与所选侧一致；没有连接 SDK/CAN 或发送复位指令。
- catkin 构建、Python 编译检查、入口脚本语法检查通过。未做真实单臂采集、推理控制或复位验收。
