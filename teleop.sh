#!/bin/bash
set -euo pipefail
source /home/robot/AgileX_ws/env.sh
exec /usr/bin/python3 /home/robot/AgileX_ws/src/model_deploy/scripts/teleop_console.py "$@"
