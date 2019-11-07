cmake_minimum_required(VERSION 2.8.3)
project(mcr_arm_motions)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    std_msgs
    geometry_msgs
    moveit_msgs
    brics_actuator
    moveit_commander
)

### TESTS
roslint_python(
  ros/scripts/cartesian_motion_node
  ros/scripts/planned_motion_node
  ros/scripts/planned_motion_with_feedback_node
)

install(PROGRAMS
  ros/scripts/cartesian_motion_node
  ros/scripts/planned_motion_node
  ros/scripts/planned_motion_with_feedback_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)