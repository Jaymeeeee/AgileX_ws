#!/usr/bin/env python3
"""Send operator commands to a running local HIL session."""

import argparse
import json
import socket
import sys


COMMANDS = ("p", "h", "hold", "stop", "q", "status")
DEFAULT_SOCKET = "/tmp/piper-act-hil.sock"


def send_command(socket_path, command):
    """Send one command and read its acknowledgement over a fresh connection."""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(5.0)
            connection.connect(socket_path)
            payload = json.dumps({"command": command}).encode("utf-8") + b"\n"
            connection.sendall(payload)
            with connection.makefile("rb") as response:
                line = response.readline(65537)
        if not line:
            raise ValueError("部署进程关闭了连接，未返回确认")
        if len(line) > 65536 or not line.endswith(b"\n"):
            raise ValueError("部署进程返回了不完整或过长的确认消息")
        acknowledgement = json.loads(line)
        if not isinstance(acknowledgement, dict):
            raise ValueError("确认消息必须是 JSON 对象")
    except (OSError, ValueError) as error:
        print(
            "命令未确认：{}。请确认部署进程正在运行，socket 路径为 {}。".format(
                error, socket_path
            ),
            file=sys.stderr,
        )
        return False

    message = acknowledgement.get("message")
    if message is None:
        message = json.dumps(acknowledgement, ensure_ascii=False)
    success = acknowledgement.get("ok", True) is not False
    print(message, file=sys.stdout if success else sys.stderr, flush=True)
    if "state" in acknowledgement:
        print("当前状态：{}".format(acknowledgement["state"]), flush=True)
    return success


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=DEFAULT_SOCKET, help="部署进程 Unix socket")
    parser.add_argument("command", nargs="?", choices=COMMANDS)
    args = parser.parse_args(argv)

    if args.command:
        return 0 if send_command(args.socket, args.command) else 1

    print("ACT 人在环控制：p 模型 / h 人工 / hold 暂停 / stop 停止 / q 退出部署")
    print("status 查询当前状态；命令提交确认不代表模式切换已完成。")
    print("输入命令后按回车；Ctrl+C 发送 stop 并退出控制台。", flush=True)
    try:
        while True:
            command = input("HIL> ").strip().lower()
            if not command:
                continue
            if command not in COMMANDS:
                print("可用命令：" + " / ".join(COMMANDS), flush=True)
                continue
            if not send_command(args.socket, command):
                return 1
            if command in ("stop", "q"):
                return 0
    except KeyboardInterrupt:
        print("\n正在发送 stop…", flush=True)
        return 0 if send_command(args.socket, "stop") else 1
    except EOFError:
        print("\n控制台输入已关闭；部署进程保持当前状态。", flush=True)
        return 0


if __name__ == "__main__":
    sys.exit(main())
