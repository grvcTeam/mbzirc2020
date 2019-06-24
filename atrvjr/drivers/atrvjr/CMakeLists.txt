cmake_minimum_required(VERSION 2.8.3)
project(atrvjr)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    rospy
)

find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

catkin_package(
 CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
