# LeRobot 仓库

`third_party/lerobot` 是独立 Git 仓库，作为个人 fork 的本地工作副本：

- `origin`：https://github.com/Jaymeeeee/lerobot.git，默认拉取和推送。
- `upstream`：https://github.com/huggingface/lerobot.git，用于获取官方更新。
- 当前 RLT 实验分支为 `test/pi05_rlt`，LeRobot 的 `main` 保留为同步基线。

2026-10-20 固定的初始 RLT 实验版本：
[`136b7b66a592a57d137bc7e5f9a0fd1cf19e7a56`](https://github.com/Jaymeeeee/lerobot/commit/136b7b66a592a57d137bc7e5f9a0fd1cf19e7a56)。
按当前实现先训练，已知论文差异暂不修正；记录在该分支的
[BASELINE.md](https://github.com/Jaymeeeee/lerobot/blob/test/pi05_rlt/examples/pi05_rlt/BASELINE.md)
和 [PAPER_AUDIT.md](https://github.com/Jaymeeeee/lerobot/blob/test/pi05_rlt/examples/pi05_rlt/PAPER_AUDIT.md)。
ROS 在线采样、主臂同步和人工接管仍需后续接入。

本机终端 Git 尚未配置 HTTPS 写入凭据；此次通过已连接的 GitHub 应用上传。
以后在终端直接推送前需完成 GitHub 凭据配置。

修改 LeRobot 后，在它的目录内检查、选择文件提交并推送：

```bash
cd ~/AgileX_ws/third_party/lerobot
git status
git diff
git add <修改的文件路径>
git commit -m "说明本次修改"
git push origin test/pi05_rlt
```

从个人 fork 拉取更新：

```bash
git pull --ff-only
```

需要同步官方更新时，先提交本地修改，再执行：

```bash
git fetch upstream
git switch main
git merge upstream/main
# 检查更新并完成相关验证；若有冲突，解决并提交后再推送。
git push origin main
git switch test/pi05_rlt
```

提交和推送不会在保存文件时自动执行。AgileX_ws 和 LeRobot 的提交分别管理；在工作空间根目录执行 `git push` 推送的是 AgileX_ws。

此目录是独立克隆，未注册为 AgileX_ws 的 submodule，已从工作空间 Git 跟踪中排除。
只克隆 AgileX_ws 不会得到 LeRobot 源码。换机器后在工作空间根目录执行：

```bash
git clone --branch test/pi05_rlt https://github.com/Jaymeeeee/lerobot.git third_party/lerobot
git -C third_party/lerobot remote add upstream https://github.com/huggingface/lerobot.git
git -C third_party/lerobot config remote.pushDefault origin
```

要精确重现上述初始版本，可再执行
`git -C third_party/lerobot checkout 136b7b66a592a57d137bc7e5f9a0fd1cf19e7a56`（进入 detached HEAD）。
继续开发则使用 `git -C third_party/lerobot switch test/pi05_rlt`。
