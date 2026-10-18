cmake_minimum_required(VERSION 3.0.2)
project(data_collection)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/bag_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Offline converter uses its own Python >= 3.12 environment. Do not let
# catkin_install_python rewrite its shebang to ROS Noetic's Python 3.8.
install(PROGRAMS scripts/bag_to_lerobot.py scripts/convert_lerobot.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md requirements-conversion.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
