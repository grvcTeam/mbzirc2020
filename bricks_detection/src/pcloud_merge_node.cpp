/*!
 *      @file  pcloud_merge_node.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  2/12/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <ros/ros.h>

#include "pcloud_merge_handler.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pcloud_merge_node");

   ROS_INFO("[%s] Node initialization.", ros::this_node::getName().c_str());

   if (!ros::master::check())
   {
      ROS_ERROR("[%s] roscore is not running.", ros::this_node::getName().c_str());
      return EXIT_FAILURE;
   }

   mbzirc::PCloudMergeHandler pcloud_merge_handler(ros::this_node::getName());

   ros::spin();

   ROS_INFO("[%s] Node finished.", ros::this_node::getName().c_str());

   return 0;
}