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

#include <std_msgs/Header.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/SetBool.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bricks_detection/types/image_item.h>
#include <bricks_detection/types/pcloud_item.h>

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
   void loadDynamicReconfigure();

   void filtersReconfigureCb(bricks_detection::reconfig_filtersConfig& config, uint32_t);

   bool usePointcloudCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

   void rgbImageCb(const sensor_msgs::Image::ConstPtr& image_msg);
   void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
   void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg);

   void publishMbzircObjectList(const std::vector<ImageItem>& detected_items,
                                const std_msgs::Header& original_header) const;
   void publishMbzircObjectList(const std::vector<PCloudItem>& detected_items,
                                const std_msgs::Header& original_header) const;

   void publishDebugPointclouds(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                const std_msgs::Header& original_header) const;

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
   ros::Publisher _detected_bricks_pub;
   ros::Publisher _detected_bricks_pose_pub;

   tf2_ros::Buffer _tf_buffer;
   tf2_ros::TransformListener* _tf_listener;

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