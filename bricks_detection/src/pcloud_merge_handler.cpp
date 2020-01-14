/*!
 *      @file  pcloud_merge_handler.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  2/12/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "pcloud_merge_handler.h"

namespace mbzirc
{
PCloudMergeHandler::PCloudMergeHandler(std::string) : _nh("~")
{
   loadParameters();
   loadServices();

   _tf_listener = new tf2_ros::TransformListener(_tf_buffer);

   loadTopics();
}

PCloudMergeHandler::~PCloudMergeHandler() {}

void PCloudMergeHandler::loadParameters()
{
   _nh.param<std::string>("tf_prefix", _tf_prefix, "");
   if (_tf_prefix != "") _tf_prefix += "/";

   _nh.param<std::string>("pcloud_sub_topic", _pcloud_sub_topic, "/camera/depth_registered/points");
   _nh.param<std::string>("pcloud_pub_topic", _pcloud_pub_topic, "merged_pcloud");

   ROS_INFO_STREAM("Parameters loaded!");
}

void PCloudMergeHandler::loadServices()
{
   _pcloud_shoot_srv   = _nh.advertiseService("shoot", &PCloudMergeHandler::shootPCloudCb, this);
   _pcloud_reset_srv   = _nh.advertiseService("reset", &PCloudMergeHandler::resetPCloudCb, this);
   _pcloud_publish_srv = _nh.advertiseService("publish", &PCloudMergeHandler::publishPCloudCb, this);
}

void PCloudMergeHandler::loadTopics()
{
   _pcloud2_sub =
       _nh.subscribe<sensor_msgs::PointCloud2>(_pcloud_sub_topic, 1, &PCloudMergeHandler::pointcloudCb, this);

   ROS_INFO_STREAM("Subscribed topics loaded!");

   _pcloud2_pub = _nh.advertise<sensor_msgs::PointCloud2>(_pcloud_pub_topic, 1);

   ROS_INFO_STREAM("Advertised topics loaded!");
}

void PCloudMergeHandler::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg)
{
   _last_pcloud2_msg = *pcloud_msg;
}

bool PCloudMergeHandler::shootPCloudCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
   ROS_INFO("New shoot! Transforming pcloud to frame %s", (_tf_prefix + "base_link").c_str());

   geometry_msgs::TransformStamped baselink_tf;
   try
   {
      baselink_tf =
          _tf_buffer.lookupTransform(_tf_prefix + "base_link", _last_pcloud2_msg.header.frame_id, ros::Time(0));
   }
   catch (tf2::TransformException& e)
   {
      ROS_WARN("%s", e.what());
      res.success = false;
      return false;
   }

   pcl::PointCloud<pcloud_type> pcloud;
   pcl::fromROSMsg(_last_pcloud2_msg, pcloud);

   pcl::PointCloud<pcloud_type> tf_pcloud;
   pcl_ros::transformPointCloud(pcloud, tf_pcloud, baselink_tf.transform);

   ROS_INFO_STREAM("Mergering pointclouds..");
   _result_pcloud2 += tf_pcloud;

   ROS_INFO_STREAM("Done!");

   res.success = true;
   return true;
}

bool PCloudMergeHandler::resetPCloudCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
   _result_pcloud2 = pcl::PointCloud<pcloud_type>();

   res.success = true;
   return true;
}

bool PCloudMergeHandler::publishPCloudCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
   pcl::PCLPointCloud2 pcloud2_out;
   pcl::toPCLPointCloud2(_result_pcloud2, pcloud2_out);

   sensor_msgs::PointCloud2 pcloud2_msg;
   pcl_conversions::fromPCL(pcloud2_out, pcloud2_msg);

   pcloud2_msg.header.stamp    = ros::Time::now();
   pcloud2_msg.header.frame_id = _tf_prefix + "base_link";

   _pcloud2_pub.publish(pcloud2_msg);

   _result_pcloud2 = pcl::PointCloud<pcloud_type>();

   res.success = true;
   return true;
}

}  // namespace mbzirc