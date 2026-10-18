# 话题与数据语义

[返回工作空间 README](../README_zh.md) · [硬件启动与消息查看](hardware.md#查看图像与消息)

## 采集模式话题

采集模式默认启动全部设备时，每台相机 **3 个话题**、每侧 Piper **8 个话题**，另有 2 个硬件状态话题，共 **27 个**。下表 `{cam}` 为 `cam_h/cam_l/cam_r`，`{side}` 为 `left/right`。

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

**力矩是电流估算值，不是力矩传感器读数或外部接触力矩**，尚未做载荷标定。已记录的固件查询结果见[验证记录](validation.md#act-部署固件查询与复位2026-10-16)。夹爪协议力字段同样未经标定。设备未安装末端六维力传感器，目前没有 `[Fx,Fy,Fz,Mx,My,Mz]` 反馈。

### 时间戳与数据含义

- 主臂话题是总线目标回显，不是主臂独立实测反馈。目标报文 ID 为 `0x155/156/157/159`，从臂关节与夹爪反馈为 `0x2A5/2A6/2A7/2A8`；CAN ID 不能区分发送同类指令的不同控制器。
- 六关节角由三帧、六电机反馈由六帧组装；各帧必须更新且默认跨度不超过 50 ms。时间戳取最新分片的主机接收时间，不代表六轴严格同时采样。`joint_states.velocity/effort` 为空，轴速度和力矩见 `motor_feedback`。
- 图像使用 SDK global/system 时间戳，对齐深度沿用原深度时间戳。深度对齐是同一相机内的空间对齐，三相机没有硬件同步。
- 相机内参及深度到彩色的外参保存在 `/{cam}/calibration` 参数、硬件状态和录制 manifest 中；不发布 camera_info 或 TF。

协议参考：[主从报文](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/double_piper.MD)、[力矩系数](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/firmware_reference.md)。查询固件可用官方 [Piper_sdk_ui](https://github.com/agilexrobotics/Piper_sdk_ui) 的 `hardware version`，记录 `S-V…` 软件版本；查询会发送 CAN 帧，需停止采集、独立连接待查机械臂并使用可发送的查询接口，完成后恢复 LISTEN-ONLY。

## 部署模式的区别

部署模式不发布主臂目标话题 `leader_targets`、`gripper/target_stamped`，其余设备按启动选择发布。上面的 27 个话题计数适用于完整采集模式，不包含部署、复位或轨迹回放另外发布的进度与动作话题。

模型预测和实际下发目标的含义见[部署输出](../src/model_deploy/README.md#查看输出)；数据集的字段顺序与重采样规则见[采集包文档](../src/data_collection/README.md#同步与溯源)。
