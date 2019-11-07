cmake_minimum_required(VERSION 2.8.3)
project(ur5_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SystemPowerSupply.msg
  SystemStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(

)

catkin_package(CATKIN_DEPENDS message_runtime)
