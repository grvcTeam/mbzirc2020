/*!
 *      @file  bricks_detection_handler.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  16/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/SetBool.h>

#include <bricks_detection/pointcloud_filtersConfig.h>

namespace mbzirc
{
class BricksDetection;
class BricksDetectionHandler
{
  public:
   BricksDetectionHandler(std::string name);
   virtual ~BricksDetectionHandler(void);

  private:
   void loadParameters();
   void loadServices();
   void loadTopics(const bool set_publishers = true);

   void filtersReconfigureCb(bricks_detection::pointcloud_filtersConfig& config, uint32_t);

   bool usePointcloudCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

   void rgbImageCb(const sensor_msgs::Image::ConstPtr& image_msg);
   void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg);

   ros::NodeHandle _nh;

   dynamic_reconfigure::Server<bricks_detection::pointcloud_filtersConfig> _pcloud_filter_rconfig_server;
   dynamic_reconfigure::Server<bricks_detection::pointcloud_filtersConfig>::CallbackType _pcloud_filter_f;

   ros::ServiceServer _use_pointcloud_srv;

   ros::Subscriber _rgb_sub;
   ros::Subscriber _pcloud2_sub;
   ros::Publisher _rgb_pub;
   ros::Publisher _pcloud2_red_pub;
   ros::Publisher _pcloud2_blue_pub;
   ros::Publisher _pcloud2_orange_pub;
   ros::Publisher _pcloud2_green_pub;
   ros::Publisher _bricks_detected_pub;

   tf::TransformListener _baselink_listener;

   std::string _image_topic;
   std::string _pcloud_topic;
   std::string _colors_json;

   BricksDetection* _bricks_detection = {nullptr};

   bool _use_pointcloud;
};
}  // namespace mbzirc