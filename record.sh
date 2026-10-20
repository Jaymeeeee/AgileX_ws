#!/bin/bash
set -e
load_ros_environment() { source /home/robot/AgileX_ws/env.sh; }
load_ros_environment
case "${1:-}" in
  --output|--output=*|--arms|--arms=*|--help|-h)
    exec rosrun data_collection bag_recorder.py "$@"
    ;;
  start|stop)
    operation="$1"
    exec rosservice call "/agilex/recording/$operation" '{}'
    ;;
  status)
    exec rostopic echo -n 1 /agilex/recording/status
    ;;
  run)
    if [ "$#" -gt 0 ]; then shift; fi
    exec roslaunch data_collection record.launch auto_start:=true "$@"
    ;;
  *)
    echo 'Usage: record.sh --output=/absolute/path/episode_directory [--arms both|left|right] [--allow-partial]' >&2
    echo 'Press Ctrl+C to save data.bag, manifest.json and rosbag.log. Reusing the directory overwrites the previous recording.' >&2
    exit 2
    ;;
esac
