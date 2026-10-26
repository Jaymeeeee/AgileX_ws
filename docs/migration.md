# 框架替换记录

本次将仓库从 ROS/catkin 工程替换为独立 Python SDK 框架，继续使用 `feat/hil-rl-sdk` 分支。活动代码只有 `piper_control/`，入口为 `scripts/` 和 `python -m piper_control`。

此前的 ROS 包、launch、CMake 文件、Shell 入口、旧文档、训练脚本和上一版 `hil_rl/` 已移出活动工程。本机完整归档为：

```text
.legacy/sdk-rewrite-20261023-100150/
```

归档保留修改后的真实工作区文件，包括未跟踪文件；没有用 Git 中的旧版本覆盖它们。350 个文件及符号链接在移动后按 SHA256 或链接目标核验。归档包含：

- `file-manifest.json`：源文件校验清单。
- `working-tree.patch`、`index.patch`、`git-status.txt`、`head.txt`：替换前 Git 状态。
- `src/`、`hil_rl/`、`docs/`、`training/`、旧入口和原 README。
- catkin 标记及源文件符号链接保持原样。原 `build/`、`devel/` 曾一并归档，随后在缓存清理中移除；恢复旧工程后需重新编译。

`.legacy/` 在本机 Git 忽略列表中，防止归档再次进入新框架。需要回看时直接阅读归档；恢复旧工程前先另存新框架，再依据归档中的 `RESTORE.txt` 和清单恢复原路径。归档不是新的运行入口。

2026-10-23 根据缓存清理请求，已删除归档中的旧构建产物，以及活动工程、LeRobot 和日志脚本的 Python／测试工具缓存。清理清单保存在归档的 `cache-cleanup.json`；原始 `file-manifest.json` 中的 350 个文件及符号链接均保留并重新校验通过。

`bags/`、`datasets/`、`outputs/`、`logs/`、已有模型文件及独立的 `third_party/lerobot` 未移动或改写。本次没有重新克隆 LeRobot，没有安装依赖，也没有执行真实机械臂动作或修改 CAN 网卡。
