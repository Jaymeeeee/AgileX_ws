#!/bin/bash
set -e
source /home/robot/AgileX_ws/env.sh
python3 -m unittest discover -s /home/robot/AgileX_ws/src/hardware_bringup/tests -p 'test_*.py' -v
python3 -m unittest discover -s /home/robot/AgileX_ws/src/data_collection/tests -p 'test_*.py' -v
