cmake_minimum_required(VERSION 3.0.2)
project(hardware_bringup)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs message_generation)
find_package(realsense2 REQUIRED CONFIG)
catkin_python_setup()
add_message_files(FILES MotorFeedback.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime)
add_executable(realsense_images src/realsense_images.cpp)
target_compile_features(realsense_images PRIVATE cxx_std_14)
target_include_directories(realsense_images PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(realsense_images ${catkin_LIBRARIES} realsense2::realsense2)
install(TARGETS realsense_images RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/hardware_manager.py
  scripts/piper_state_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
