cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_generation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
)


### TESTS
roslint_python(
  ros/scripts/sampling_parameter_gui_node
)


install(PROGRAMS
  ros/scripts/sampling_parameter_gui_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
