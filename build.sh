#!/bin/bash
set -e
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/noetic/setup.bash
exec catkin_make -C /home/robot/AgileX_ws -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_ENABLE_TESTING=OFF "$@"
