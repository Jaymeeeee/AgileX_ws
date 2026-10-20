# 硬件启动与运行模式

[返回工作空间 README](../README.md) · [话题与数据语义](topics.md)

设备映射见[硬件与运行模式](../README.md#硬件与运行模式)，具体参数以 [hardware.yaml](../src/hardware_bringup/config/hardware.yaml) 为准。相机按序列号绑定名称；USB-CAN 重插后需确认左右接口映射。

## 采集模式启动与停止

数据采集使用四臂软件主从控制，关闭占用相机的 RealSense Viewer 或其他驱动后执行：

```bash
~/AgileX_ws/prepare_can.sh --mode=teleop
~/AgileX_ws/teleop.sh --execute --cameras=all
# 输入 d，完成夹爪接管且 READY 后再启动 record.sh。
```

当前为四路独立 CAN：左主 can1、右主 can2、左从 can3、右从 can0，并校验 USB 物理路径。
`prepare_can.sh` 需要本机 sudo 权限，将接口设为 1 Mbps、正常收发、UP，配置已正确时跳过。
独立单臂需要电脑提供 CAN ACK，因此只读发布也不再使用 LISTEN-ONLY。
软件主从及录制使用[teleop.sh](teleop.md)，仅观察反馈时才单独启动本页的硬件管理器。

采集终端输入 `r` 后按回车，可保存当前录制并复位四臂；详见[复位动作与操作键](teleop.md#启动)。
若只需发布反馈或单独检查设备，才使用下面的 `start_hardware.sh`，不要与 `teleop.sh` 同时启动：

```bash
# 仅三台相机
~/AgileX_ws/start_hardware.sh arms:=none
# 仅左臂相机
~/AgileX_ws/start_hardware.sh arms:=none cameras:=cam_l
# 仅两台从臂反馈
~/AgileX_ws/start_hardware.sh cameras:=none
# 仅右从臂反馈
~/AgileX_ws/start_hardware.sh arms:=right cameras:=none
```

`arms` 支持 `all`、`none`、`left`、`right` 或逗号列表；`cameras` 支持 `all`、`none` 或相机名称的逗号列表。`fps:=30` 可覆盖帧率，但需另行验证带宽与稳定性。

- `READY`：所选设备的话题检查通过，终端输出话题、内容摘要和帧率。
- `DEGRADED`：尚未收到主臂目标时，继续发布已有反馈和图像，但不允许录制。加 `require_leader_targets:=true` 可要求主臂目标在启动超时前就绪。
- 启动失败、必需连续流中断或驱动退出：报告原因并关闭本管理器启动的驱动。

采集模式每次启动后，所选侧的主臂关节和夹爪目标都需至少收到一次。静止时目标可能不再广播，节点保留最后值及原始时间戳；默认不因目标变旧报错，因此缓存不能证明主臂仍在线。超时策略见 `monitor.leader_target_timeout_sec`。

停止时先结束录制，再在硬件启动终端按 **Ctrl+C**，或执行：

```bash
source ~/AgileX_ws/env.sh
rosnode kill /agilex_hardware
```

停止 ROS 发布不等同于机械臂断电或急停。不要重复启动管理器。

## 切换部署模式与恢复采集

操作层分为采集、一般部署、RL 人在环三个模式，最后一个尚未实现，见[模式表](../README.md#硬件与运行模式)。
硬件管理器内部 record/deploy 都使用正常 CAN。管理器本身仍只接收，不使能或驱动机械臂；
record 模式的人工作用由独立主臂节点发布，deploy 模式由模型/复位/回放控制器发送。
切换前先结束录制及当前控制进程，再停止旧硬件管理器。保持四臂物理隔离，不恢复主从直连。

```bash
~/AgileX_ws/prepare_can.sh --mode=deploy
~/AgileX_ws/start_hardware.sh mode:=deploy
```

普通部署、独立复位和回放只需要两台从臂通电，主臂可断电；`prepare_can.sh --mode=deploy` 默认仅准备从臂 can3/can0。仅复位或实机轨迹回放时可加 `cameras:=none`。部署模式不发布 `leader_targets` 和 `gripper/target_stamped`，避免把电脑控制回显作为主臂示范；回显仍可在 `events` 中追溯并标记来源。该模式不要求主臂目标就绪。

恢复采集前，停止 SDK 控制程序和硬件管理器，再执行：

```bash
~/AgileX_ws/prepare_can.sh --mode=teleop
~/AgileX_ws/teleop.sh --execute --cameras=all
```

`prepare_can.sh` 默认使用 `record`，只在配置已经正确或接口未启用时准备接口；接口正在以其他配置运行时，需要显式指定 `--mode=record` 或 `--mode=deploy`。脚本会检查硬件桥接进程及 SDK 控制锁，拒绝在它们占用接口时重新配置。此脚本只配置接口，不发送使能、复位或运动报文。

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

## 进一步操作

- [录制与 bag 消息回放](../src/data_collection/README.md#录制-rosbag)：保存数据、检查完整性及重新发布话题。
- [模型部署、复位与实机轨迹回放](../src/model_deploy/README.md)：SDK 控制、停止处理及运行日志。
- [话题与数据语义](topics.md)：完整话题表、单位、时间戳、力矩估算及标定信息。
