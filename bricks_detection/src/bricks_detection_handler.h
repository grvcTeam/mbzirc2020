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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/SetBool.h>

#include <bricks_detection/reconfig_filtersConfig.h>

namespace mbzirc
{
struct CameraParameters
{
   tf2::Matrix3x3 K;
   tf2::Matrix3x3 R;
   tf2::Vector3 T;
};

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

   void filtersReconfigureCb(bricks_detection::reconfig_filtersConfig& config, uint32_t);

   bool usePointcloudCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

   void rgbImageCb(const sensor_msgs::Image::ConstPtr& image_msg);
   void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
   void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg);

   ros::NodeHandle _nh;

   dynamic_reconfigure::Server<bricks_detection::reconfig_filtersConfig> _pcloud_filter_rconfig_server;
   dynamic_reconfigure::Server<bricks_detection::reconfig_filtersConfig>::CallbackType _pcloud_filter_f;

   ros::ServiceServer _use_pointcloud_srv;

   ros::Subscriber _rgb_sub;
   ros::Subscriber _rgb_info_sub;
   ros::Subscriber _pcloud2_sub;
   ros::Publisher _rgb_pub;
   ros::Publisher _pcloud2_red_pub;
   ros::Publisher _pcloud2_blue_pub;
   ros::Publisher _pcloud2_orange_pub;
   ros::Publisher _pcloud2_green_pub;
   ros::Publisher _bricks_detected_pub;

   tf2_ros::Buffer _tf_buffer;
   tf2_ros::TransformListener* _tfListener;

   std::string _image_topic;
   std::string _image_info_topic;
   std::string _pcloud_topic;
   std::string _colors_json;
   std::string _tf_prefix;

   bool _use_pointcloud;

   BricksDetection* _bricks_detection = {nullptr};
   CameraParameters _camera_parameters;
};
}  // namespace mbzirc