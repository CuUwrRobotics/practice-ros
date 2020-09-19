# @Author: Nick Steele <nichlock>
# @Date:   15:24 Sep 08 2020

cmake_minimum_required(VERSION 2.8.3)
project(py_subscriber)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
 INCLUDE_DIRS src ${catkin_INCLUDE_DIRS}
 CATKIN_DEPENDS
 rospy
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
