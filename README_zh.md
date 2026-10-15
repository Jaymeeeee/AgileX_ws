# AgileX_ws：双组 Piper 主从臂与三台 D435

ROS 1 Noetic 工作空间，用于硬件状态发布和 rosbag 采集。机械臂 CAN 采用只接收模式，不发送使能、回零或运动指令；模型部署尚未实现。

## 包与配置

| 包 | 功能 |
| --- | --- |
| `hardware_bringup` | 启动相机、解析 CAN、发布硬件数据和就绪状态 |
| `data_collection` | 录制 rosbag、无损压缩、保存配置并检查录制结果 |
| `model_deploy` | 模型部署占位包 |

- [hardware.yaml](src/hardware_bringup/config/hardware.yaml)：设备映射、相机参数和健康检查阈值。
- [recording.yaml](src/data_collection/config/recording.yaml)：录制路径、压缩、分包和磁盘阈值。
- `bags/`：录制数据；`logs/`：测试和运行记录。

依赖 ROS Noetic、RealSense C++ SDK、python-can 和 pyAgxArm 1.0.0。入口脚本自动加载环境，使用系统 Python 3.8。直接运行 ROS 命令前执行：

```bash
source ~/AgileX_ws/env.sh
```

修改代码后编译、检查：

```bash
~/AgileX_ws/build.sh
~/AgileX_ws/test.sh
```

## 设备映射

| 设备 | 名称 / 通道 | 相机序列号 |
| --- | --- | --- |
| 第三视角相机 | `cam_h` | `262522071923` |
| 左臂相机 | `cam_l` | `262522071867` |
| 右臂相机 | `cam_r` | `262522070143` |
| 左组主从臂 | `can0` | — |
| 右组主从臂 | `can1` | — |

相机按 SDK 序列号绑定名称，默认 640×480、15 FPS，发布彩色图、深度图及对齐深度图。USB-CAN 重插后需确认左右接口映射。

## 启动与停止

关闭占用相机的 RealSense Viewer 或其他驱动，然后准备 CAN 并启动硬件：

```bash
~/AgileX_ws/prepare_can.sh
~/AgileX_ws/start_hardware.sh
```

`prepare_can.sh` 需要本机 sudo 权限，将 DOWN 接口设为 1 Mbps、LISTEN-ONLY、UP；配置已正确时跳过，正在以其他模式运行时拒绝切换。可指定单口：`~/AgileX_ws/prepare_can.sh can0`。

常用启动选项：

```bash
# 仅三台相机
~/AgileX_ws/start_hardware.sh arms:=none
# 仅左臂相机
~/AgileX_ws/start_hardware.sh arms:=none cameras:=cam_l
# 仅两组机械臂
~/AgileX_ws/start_hardware.sh cameras:=none
# 仅右组机械臂
~/AgileX_ws/start_hardware.sh arms:=right cameras:=none
```

`arms` 支持 `all`、`none`、`left`、`right` 或逗号列表；`cameras` 支持 `all`、`none` 或相机名称的逗号列表。`fps:=30` 可覆盖帧率，但需另行验证带宽与稳定性。

- `READY`：所选设备的话题检查通过，终端输出话题、内容摘要和帧率。
- `DEGRADED`：尚未收到主臂目标时，继续发布已有反馈和图像，但不允许录制。加 `require_leader_targets:=true` 可要求主臂目标在启动超时前就绪。
- 启动失败、必需连续流中断或驱动退出：报告原因并关闭本管理器启动的驱动。

每次启动后，两侧主臂关节和夹爪目标都需至少收到一次。静止时目标可能不再广播，节点保留最后值及原始时间戳；默认不因目标变旧报错，因此缓存不能证明主臂仍在线。超时策略见 `monitor.leader_target_timeout_sec`。

停止时先结束录制，再在硬件启动终端按 **Ctrl+C**，或执行：

```bash
source ~/AgileX_ws/env.sh
rosnode kill /agilex_hardware
```

停止 ROS 发布不等同于机械臂断电或急停。不要重复启动管理器。

## 发布话题

每台相机 **3 个话题**、每侧 Piper **8 个话题**，另有 2 个硬件状态话题，共 **27 个**。下表 `{cam}` 为 `cam_h/cam_l/cam_r`，`{side}` 为 `left/right`。

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/{cam}/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图，rgb8 |
| `/{cam}/depth/image_rect_raw` | `sensor_msgs/Image` | 原始深度，16UC1，mm，0 无效 |
| `/{cam}/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 重投影到彩色图像坐标的深度，16UC1，mm，0 无效 |
| `/piper/{side}_bus/joint_states` | `sensor_msgs/JointState` | 从臂六关节实际角度，rad，顺序见 name |
| `/piper/{side}_bus/leader_targets` | `sensor_msgs/JointState` | 六关节目标指令回显，rad |
| `/piper/{side}_bus/gripper/actual_stamped` | `sensor_msgs/JointState` | 从臂夹爪实际开口，position[0]，m |
| `/piper/{side}_bus/gripper/target_stamped` | `sensor_msgs/JointState` | 夹爪目标开口回显，m |
| `/piper/{side}_bus/status` | `std_msgs/String`（JSON） | 控制模式、运动/示教状态、故障码和原始 CAN 数据 |
| `/piper/{side}_bus/gripper/status` | `std_msgs/String`（JSON） | 夹爪开口、状态位、协议力字段和原始 CAN 数据 |
| `/piper/{side}_bus/motor_feedback` | `hardware_bringup/MotorFeedback` | 六电机电流、轴位置/速度和力矩估算 |
| `/piper/{side}_bus/events` | `std_msgs/String`（JSON） | 解码事件、原始 CAN 字节和各分片接收时间；不是全部 CAN 报文录制 |
| `/agilex/hardware/status` | `std_msgs/String`（JSON） | 就绪状态、配置、标定、各话题有效性/年龄/帧率 |
| `/agilex/hardware/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ROS 诊断 |

### 电机反馈与力矩

`motor_feedback` 约 100 Hz，字段如下：

| 字段 | 含义 |
| --- | --- |
| `header` | 六帧中最新的 CAN 主机接收时间戳 |
| `joint_order` | 数组关节顺序 `[1, 2, 3, 4, 5, 6]` |
| `current_a` | 六电机电流，A |
| `motor_position_rad` | SDK 解码的电机轴位置，rad |
| `motor_velocity_rad_s` | SDK 解码的电机轴速度，rad/s |
| `torque_estimate_nm` | 六关节电流换算力矩，N·m；型号未知时为空 |

标准 Piper 换算公式为 `torque = current × coefficient`：J1–3 系数 4.725，J4–6 系数 0.7787701152 N·m/A。型号、系数、原始报文和分片时间戳保存在同侧 `events` 中。

**力矩是电流估算值，不是力矩传感器读数或外部接触力矩**，尚未做载荷标定，固件版本待核对。夹爪协议力字段同样未经标定。设备未安装末端六维力传感器，目前没有 `[Fx,Fy,Fz,Mx,My,Mz]` 反馈。

### 时间戳与数据含义

- 主臂话题是总线目标回显，不是主臂独立实测反馈。目标报文 ID 为 `0x155/156/157/159`，从臂关节与夹爪反馈为 `0x2A5/2A6/2A7/2A8`；CAN ID 不能区分发送同类指令的不同控制器。
- 六关节角由三帧、六电机反馈由六帧组装；各帧必须更新且默认跨度不超过 50 ms。时间戳取最新分片的主机接收时间，不代表六轴严格同时采样。`joint_states.velocity/effort` 为空，轴速度和力矩见 `motor_feedback`。
- 图像使用 SDK global/system 时间戳，对齐深度沿用原深度时间戳。深度对齐是同一相机内的空间对齐，三相机没有硬件同步。
- 相机内参及深度到彩色的外参保存在 `/{cam}/calibration` 参数、硬件状态和录制 manifest 中；不发布 camera_info 或 TF。

协议参考：[主从报文](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/double_piper.MD)、[力矩系数](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/firmware_reference.md)。查询固件可用官方 [Piper_sdk_ui](https://github.com/agilexrobotics/Piper_sdk_ui) 的 `hardware version`，记录 `S-V…` 软件版本；查询会发送 CAN 帧，需停止采集、独立连接待查机械臂并使用可发送的查询接口，完成后恢复 LISTEN-ONLY。

## 查看图像与消息

保持硬件运行，在另一终端执行：

```bash
source ~/AgileX_ws/env.sh
rosrun rqt_image_view rqt_image_view /cam_h/color/image_raw
```

在下拉框选择其他相机或深度话题。不要同时用 RealSense Viewer 打开同一相机。

```bash
rostopic list
rostopic echo -n 1 /piper/left_bus/joint_states
rostopic echo -n 1 /piper/left_bus/leader_targets
rostopic echo -n 1 /piper/left_bus/gripper/actual_stamped
rostopic echo -n 1 /piper/left_bus/status
rostopic echo -n 1 /piper/left_bus/motor_feedback
rostopic echo -n 1 /piper/left_bus/motor_feedback/torque_estimate_nm
rostopic echo -n 1 /cam_h/depth/image_rect_raw/header
rosparam get /cam_h/calibration
rostopic echo -n 1 /agilex/hardware/diagnostics
rostopic hz /cam_h/color/image_raw
```

可替换 `left/right` 和相机名称；`rostopic hz` 按 Ctrl+C 退出。图像建议查看消息头或用图形工具显示，避免打印完整像素数组。

## 录制与回放

终端 1 启动硬件并等待 `READY`。终端 2 指定完整文件名，执行后开始录制：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/task_001.bag
```

看到 `Recording ...` 后开始演示；任务结束时在终端 2 按 **Ctrl+C**，等待 `complete` 和命令退出，完成文件保存。下一次修改文件名重新运行：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/task_002.bag
# 含空格的路径用引号包住
~/AgileX_ws/record.sh --output="/home/robot/AgileX_ws/bags/pick cube/episode_003.bag"
```

`--output` 必填，文件名以 `.bag` 结尾，支持绝对路径、相对路径和 `~`。父目录自动创建；**同名 bag 会在硬件就绪、磁盘检查通过后自动覆盖**，对应 `.metadata` 中的清单和日志同步更新。采集不满意时，按 Ctrl+C 停止后再次运行同一条命令即可重录；需要保留上一段时请换文件名。已有 `.active` 文件时拒绝覆盖。一次运行录制一段，使用 LZ4 无损压缩，**不自动分包**。同一 ROS master 同时只允许一个本包录制进程。

输出示例：

```text
bags/
├── task_001.bag
└── task_001.bag.metadata/
    ├── manifest.json    # 设备、配置、标定、起止时间、话题数量与录制结果
    └── rosbag.log       # 底层录制日志
```

默认要求两组机械臂及三台相机全部就绪；未就绪时最多等待 120 秒，超时退出。若硬件只启动了部分设备，追加 `--allow-partial`，所选设备仍须就绪：

```bash
~/AgileX_ws/record.sh --output=~/AgileX_ws/bags/cameras_001.bag --allow-partial
```

默认至少保留 5 GiB 磁盘空间。硬件异常、心跳过期、配置变化、磁盘不足或录制进程退出会结束录制并标为 `incomplete`。`complete` 表示 bag 可读、必需话题有消息、无遗留 `.active` 且未检测到缓冲溢出，不保证无任何丢帧或时序误差。先停止录制，再关闭硬件。

消息保留原始 header，bag 时间为录制端接收时间；静止主臂目标保留最后值及原时间戳。不做插值、重采样、跨设备同步或训练格式转换。

```bash
source ~/AgileX_ws/env.sh
rosbag info ~/AgileX_ws/bags/task_001.bag
# 先停止在线硬件，再回放，避免同名话题混流
rosbag play ~/AgileX_ws/bags/task_001.bag
```

## 验证记录

双组机械臂和三相机已完成约 20 秒实机录制，27 个预期话题均有数据，并通过逐条读取检查。结果见 [test.validation.json](bags/test.validation.json)，原始文件为 `bags/test.bag`。这属于短时功能验证，长期稳定性和训练数据时序仍需单独评估。其他测试记录见 [docs/validation.md](docs/validation.md)。
