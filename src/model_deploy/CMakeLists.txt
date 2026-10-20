cmake_minimum_required(VERSION 3.0.2)
project(model_deploy)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs std_srvs hardware_bringup)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs std_srvs hardware_bringup)
# Keep each script's interpreter: worker needs Python 3.12, ROS parent needs 3.8.
install(PROGRAMS scripts/deploy_node.py scripts/inference_worker.py scripts/reset_node.py scripts/replay_node.py scripts/piper_teleop_node.py scripts/teleop_console.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
