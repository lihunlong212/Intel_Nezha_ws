cmake_minimum_required(VERSION 3.0.2)
project(cargo_handler)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  rospy
)

## Generate messages
add_message_files(
  FILES
  CargoCmd.msg
  CargoDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    rospy
)

## Mark scripts as executable
catkin_install_python(PROGRAMS
  scripts/cargo_vision_node.py
  scripts/grasp_delivery_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
