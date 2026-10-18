# LeRobot 仓库

`third_party/lerobot` 是独立 Git 仓库，作为个人 fork 的本地工作副本：

- `origin`：https://github.com/Jaymeeeee/lerobot.git，默认拉取和推送。
- `upstream`：https://github.com/huggingface/lerobot.git，用于获取官方更新。
- 本地 `main` 跟踪 `origin/main`。

本机尚未配置 GitHub 写入认证，推送前需在本机完成 GitHub 的 HTTPS 凭据配置（个人访问令牌或凭据管理器）。读取公开仓库不需要登录，但不能据此确认拥有推送权限。

修改 LeRobot 后，在它的目录内检查、选择文件提交并推送：

```bash
cd ~/AgileX_ws/third_party/lerobot
git status
git diff
git add <修改的文件路径>
git commit -m "说明本次修改"
git push
```

从个人 fork 拉取更新：

```bash
git pull --ff-only
```

需要同步官方更新时，先提交本地修改，再执行：

```bash
git fetch upstream
git merge upstream/main
# 检查更新并完成相关验证；若有冲突，解决并提交后再推送。
git push origin main
```

提交和推送需要手动执行，不会在保存文件时自动上传。AgileX_ws 和 LeRobot 的提交分别管理；在工作空间根目录执行 `git push` 推送的是 AgileX_ws。

此目录当前是独立克隆，未注册为 AgileX_ws 的 submodule。Git 远程配置仅保存在本机；换机器时可以重新克隆并配置：

```bash
git clone https://github.com/Jaymeeeee/lerobot.git third_party/lerobot
git -C third_party/lerobot remote add upstream https://github.com/huggingface/lerobot.git
git -C third_party/lerobot config remote.pushDefault origin
```
