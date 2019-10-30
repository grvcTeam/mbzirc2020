/*!
 *      @file  bricks_detection_handler.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  16/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

#include <bricks_detection/bricks_detection.h>
#include "bricks_detection_handler.h"

namespace mbzirc
{
BricksDetectionHandler::BricksDetectionHandler(std::string) : _nh("~"), _use_pointcloud(false)
{
   loadParameters();
   loadServices();
   loadTopics();

   _pcloud_filter_f = boost::bind(&BricksDetectionHandler::filtersReconfigureCb, this, _1, _2);
   _pcloud_filter_rconfig_server.setCallback(_pcloud_filter_f);
}

BricksDetectionHandler::~BricksDetectionHandler() {}

void BricksDetectionHandler::loadParameters()
{
   _nh.param<bool>("use_pointcloud", _use_pointcloud, false);

   _nh.param<std::string>("rgb_img_topic", _image_topic, "/camera/color/image_rect_color");
   _nh.param<std::string>("pcloud_topic", _pcloud_topic, "/camera/depth_registered/points");

   _nh.param<std::string>(
       "colors_json", _colors_json,
       "/home/rcaballero/Projects/Mbzirc2020/team_ws/src/mbzirc2020/bricks_detection/cfg/grvc_field_testing.json");

   ROS_INFO_STREAM("Parameters loaded!");
}

void BricksDetectionHandler::loadServices()
{
   _use_pointcloud_srv = _nh.advertiseService("use_pointcloud", &BricksDetectionHandler::usePointcloudCb, this);
}

void BricksDetectionHandler::loadTopics(const bool set_publishers)
{
   if (_use_pointcloud)
   {
      _pcloud2_sub =
          _nh.subscribe<sensor_msgs::PointCloud2>(_pcloud_topic, 1, &BricksDetectionHandler::pointcloudCb, this);
   }
   else
   {
      _rgb_sub = _nh.subscribe<sensor_msgs::Image>(_image_topic, 1, &BricksDetectionHandler::rgbImageCb, this);
   }

   ROS_INFO_STREAM("Subscribed topics loaded!");

   if (set_publishers)
   {
      _rgb_pub = _nh.advertise<sensor_msgs::Image>("bricks/filtered_image", 1);

      _pcloud2_red_pub    = _nh.advertise<sensor_msgs::PointCloud2>("bricks/red", 1);
      _pcloud2_blue_pub   = _nh.advertise<sensor_msgs::PointCloud2>("bricks/blue", 1);
      _pcloud2_orange_pub = _nh.advertise<sensor_msgs::PointCloud2>("bricks/orange", 1);
      _pcloud2_green_pub  = _nh.advertise<sensor_msgs::PointCloud2>("bricks/green", 1);

      _bricks_detected_pub = _nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("bricks", 1);

      ROS_INFO_STREAM("Advertised topics loaded!");
   }
}

void BricksDetectionHandler::filtersReconfigureCb(bricks_detection::pointcloud_filtersConfig& config, uint32_t)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   if (config.colors_json_path != "default") _bricks_detection->color_filtering->addHSVFilter(config.colors_json_path);

   _bricks_detection->color_filtering->setMinPointsPerColor(config.min_points_per_color);

   _bricks_detection->distance_filtering->toggleFilters(config.enable_voxel_grid, config.enable_distance_filter,
                                                        config.enable_sor);
   if (config.enable_voxel_grid)
   {
      _bricks_detection->distance_filtering->voxel_grid.setLeafSize(config.voxel_leaf_size, config.voxel_leaf_size,
                                                                    config.voxel_leaf_size);
   }

   if (config.enable_distance_filter)
   {
      _bricks_detection->distance_filtering->z_filtering.setFilterLimits(config.min_distance_limit,
                                                                         config.max_distance_limit);
   }

   if (config.enable_sor)
   {
      _bricks_detection->distance_filtering->sor_filtering.setMeanK(config.sor_mean_k);
      _bricks_detection->distance_filtering->sor_filtering.setStddevMulThresh(config.sor_std_dev_mul_thresh);
   }

   _bricks_detection->plane_detector->toggle(config.enable_plane_segmentation);
   _bricks_detection->plane_detector->setMaxIterations(config.max_ransac_iterations);
   _bricks_detection->plane_detector->setMaxCoefs(config.plane_coef0, config.plane_coef1, config.plane_coef2);
}

bool BricksDetectionHandler::usePointcloudCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
   _use_pointcloud = req.data;

   _rgb_sub.shutdown();
   _pcloud2_sub.shutdown();
   loadTopics(false);

   ROS_INFO("Setting use_pointcloud to %s", _use_pointcloud ? "true" : "false");

   res.success = true;
   return true;
}

void BricksDetectionHandler::rgbImageCb(const sensor_msgs::Image::ConstPtr& image_msg)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   cv_bridge::CvImagePtr image_ptr;
   try
   {
      image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("cv_bridge exception: %s", e.what());
      return;
   }

   cv::Mat filtered_img;
   _bricks_detection->processData(image_ptr->image, filtered_img);

   cv_bridge::CvImage cv_bridge = cv_bridge::CvImage(image_msg->header, "bgr8", filtered_img);
   sensor_msgs::Image filtered_img_msg;
   cv_bridge.toImageMsg(filtered_img_msg);
   _rgb_pub.publish(filtered_img_msg);
}

void BricksDetectionHandler::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   tf::StampedTransform transform;
   try
   {
      _baselink_listener.lookupTransform("base_link", pcloud_msg->header.frame_id, ros::Time(0), transform);
   }
   catch (const tf::TransformException& e)
   {
      ROS_ERROR("%s", e.what());
      return;
   }

   pcl::PointCloud<pcl::PointXYZRGB> pcloud;
   pcl::fromROSMsg(*pcloud_msg, pcloud);

   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> color_pcloud_cluster;
   _bricks_detection->processData(pcloud, color_pcloud_cluster, transform);

   for (auto color_pcloud : color_pcloud_cluster)
   {
      pcl::PCLPointCloud2 pcloud2_out;
      pcl::toPCLPointCloud2(color_pcloud.second, pcloud2_out);

      sensor_msgs::PointCloud2 pcloud2_msg;
      pcl_conversions::fromPCL(pcloud2_out, pcloud2_msg);

      pcloud2_msg.header.stamp    = pcloud_msg->header.stamp;
      pcloud2_msg.header.frame_id = "base_link";

      if (color_pcloud.first == "red")
      {
         _pcloud2_red_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "blue")
      {
         _pcloud2_blue_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "orange")
      {
         _pcloud2_orange_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "green")
      {
         _pcloud2_green_pub.publish(pcloud2_msg);
         continue;
      }
   }
}
}  // namespace mbzirc