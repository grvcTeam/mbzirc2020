/*!
 *      @file  pcloud_merge_handler.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  2/12/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Trigger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mbzirc
{
using pcloud_type = pcl::PointXYZ;

class PCloudMergeHandler
{
  public:
   PCloudMergeHandler(std::string name);
   virtual ~PCloudMergeHandler(void);

  private:
   void loadParameters();
   void loadServices();
   void loadTopics();

   void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg);
   bool shootPCloudCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
   bool resetPCloudCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
   bool publishPCloudCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

   ros::NodeHandle _nh;

   ros::Subscriber _pcloud2_sub;
   ros::Publisher _pcloud2_pub;

   ros::ServiceServer _pcloud_shoot_srv;
   ros::ServiceServer _pcloud_reset_srv;
   ros::ServiceServer _pcloud_publish_srv;

   tf2_ros::Buffer _tf_buffer;
   tf2_ros::TransformListener* _tf_listener;

   std::string _pcloud_sub_topic;
   std::string _pcloud_pub_topic;

   std::string _tf_prefix;

   sensor_msgs::PointCloud2 _last_pcloud2_msg;

   pcl::PointCloud<pcloud_type> _result_pcloud2;
};
}  // namespace mbzirc