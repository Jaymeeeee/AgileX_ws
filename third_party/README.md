# LeRobot 模型依赖

`third_party/lerobot` 是独立 Git 仓库，未纳入本工程的 Python 包或 Git 跟踪。框架替换没有修改或重新克隆它。

普通部署通过 `policy.python` 使用已安装的 LeRobot；也可将 `policy.lerobot_source` 指向已有源码目录。所有 checkpoint、tokenizer 和基础模型资源须提前准备好，运行时不自动下载。

当前 π0.5/RLT 接口对应个人 fork 的 `test/pi05_rlt` 分支，核对版本为 [`136b7b66a592a57d137bc7e5f9a0fd1cf19e7a56`](https://github.com/Jaymeeeee/lerobot/commit/136b7b66a592a57d137bc7e5f9a0fd1cf19e7a56)。上游普通 LeRobot 不包含这个自定义 `pi05_rlt` 策略和 learner。

- 模型来源：[个人 fork](https://github.com/Jaymeeeee/lerobot)、[LeRobot 上游](https://github.com/huggingface/lerobot)。
- 已有基线约束：[BASELINE.md](https://github.com/Jaymeeeee/lerobot/blob/test/pi05_rlt/examples/pi05_rlt/BASELINE.md)、[PAPER_AUDIT.md](https://github.com/Jaymeeeee/lerobot/blob/test/pi05_rlt/examples/pi05_rlt/PAPER_AUDIT.md)。
- 本工程负责 SDK 执行、人工接管、实际动作采样及回合调度；learner 和模型结构由该依赖提供。

更换 LeRobot 版本后重新验证注册模型、checkpoint 处理器和 RLT 回放契约。本仓库与 LeRobot 仓库分别管理提交，保存 Python 文件不会自动提交或推送。
