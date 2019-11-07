cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_measurers)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
    message_runtime
)

### TESTS
roslint_python(
  ros/scripts/joint_distance_measurer_node
)


install(PROGRAMS
  ros/scripts/joint_distance_measurer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
