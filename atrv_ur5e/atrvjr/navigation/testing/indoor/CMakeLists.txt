cmake_minimum_required(VERSION 2.8.3)
project(indoor)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/world_odom_transf scripts/world_odom_rl_transf scripts/print_pose_error
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
