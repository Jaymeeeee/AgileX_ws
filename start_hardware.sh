#!/bin/bash
set -e
source /home/robot/AgileX_ws/env.sh
exec roslaunch hardware_bringup hardware.launch "$@"
