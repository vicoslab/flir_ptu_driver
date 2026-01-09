cmake_minimum_required(VERSION 3.10)
project(flir_ptu_driver)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy geometry_msgs std_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/ptu_node.py
  src/twist_demo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)