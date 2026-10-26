#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lerobot_python="${LEROBOT_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python}"
if [[ ! -x "$lerobot_python" ]]; then
    echo "找不到 LeRobot Python: $lerobot_python；请设置 LEROBOT_PYTHON" >&2
    exit 1
fi
lerobot_prefix="$(cd -- "$(dirname -- "$lerobot_python")/.." && pwd)"
# ROS Noetic's Python 3.8 modules must not enter the LeRobot Python 3.12 process.
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$lerobot_prefix/lib"
export PATH="$lerobot_prefix/bin:$PATH"
exec "$lerobot_python" "$script_dir/bag_to_lerobot.py" "$@"
