cmake_minimum_required(VERSION 2.8.3)
project(ch1_score_manager)

find_package(catkin REQUIRED COMPONENTS
   rospy gazebo_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy gazebo_msgs)