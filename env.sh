#!/bin/bash
export PATH="/usr/bin:/bin:$PATH"
# Isolate positional arguments: catkin's setup.sh otherwise parses --help,
# --execute or mode:=deploy as its own command-line options.
_agilex_source_ros() {
  source /opt/ros/noetic/setup.bash
  source /home/robot/AgileX_ws/devel/setup.bash
}
_agilex_source_ros
unset -f _agilex_source_ros
