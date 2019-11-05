/*!
 * @file moving_object_detector_node.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 29/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <ros/ros.h>

#include <drone_detector_tracking/moving_object_detector_handler.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "moving_object_detect");

   ROS_INFO("[%s] Node initialization.", ros::this_node::getName().c_str());

   if (!ros::master::check())
   {
      ROS_ERROR("[%s] roscore is not running.", ros::this_node::getName().c_str());
      return EXIT_FAILURE;
   }

   mbzirc::MovingObjectDetectorHandler moving_object_detector_handler(ros::this_node::getName());

   ros::spin();

   ROS_INFO("[%s] Node finished.", ros::this_node::getName().c_str());

   return 0;
}