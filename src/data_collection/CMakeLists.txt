cmake_minimum_required(VERSION 3.0.2)
project(data_collection)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/bag_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
