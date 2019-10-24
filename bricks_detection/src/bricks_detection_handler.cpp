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
BricksDetectionHandler::BricksDetectionHandler(std::string) : _nh("~")
{
   loadParameters();
   loadTopics();

   _pcloud_filter_f = boost::bind(&BricksDetectionHandler::filters_reconfigure, this, _1, _2);
   _pcloud_filter_rconfig_server.setCallback(_pcloud_filter_f);
}

BricksDetectionHandler::~BricksDetectionHandler() {}

void BricksDetectionHandler::loadParameters()
{
   _nh.param<std::string>("rgb_img_topic", _image_topic, "/camera/color/image_raw");
   _nh.param<std::string>("pcloud_topic", _pcloud_topic, "/camera/depth_registered/points");

   _nh.param<std::string>(
       "colors_json", _colors_json,
       "/home/rcaballero/Projects/Mbzirc2020/team_ws/src/mbzirc2020/bricks_detection/cfg/grvc_field_testing.json");

   ROS_INFO_STREAM("Parameters loaded!");
}

void BricksDetectionHandler::loadTopics()
{
   _pcloud2_sub =
       _nh.subscribe<sensor_msgs::PointCloud2>(_pcloud_topic, 1, &BricksDetectionHandler::pointcloudCb, this);

   _pcloud2_red_pub    = _nh.advertise<sensor_msgs::PointCloud2>("bricks/red", 1);
   _pcloud2_blue_pub   = _nh.advertise<sensor_msgs::PointCloud2>("bricks/blue", 1);
   _pcloud2_orange_pub = _nh.advertise<sensor_msgs::PointCloud2>("bricks/orange", 1);
   _pcloud2_green_pub  = _nh.advertise<sensor_msgs::PointCloud2>("bricks/green", 1);

   _bricks_detected_pub = _nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("bricks", 1);

   ROS_INFO_STREAM("Topics loaded!");
}

void BricksDetectionHandler::filters_reconfigure(bricks_detection::pointcloud_filtersConfig& config, uint32_t)
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
}

void BricksDetectionHandler::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   // transforming
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

   // processing
   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> pcloud_color_cluster;
   _bricks_detection->processData(pcloud, pcloud_color_cluster, transform);

   // publishing
   for (auto color_pcloud : pcloud_color_cluster)
   {
      pcl::PCLPointCloud2 pcloud2_out;
      pcl::toPCLPointCloud2(color_pcloud.second, pcloud2_out);

      sensor_msgs::PointCloud2 pcloud2_msg;
      pcl_conversions::fromPCL(pcloud2_out, pcloud2_msg);

      pcloud2_msg.header.stamp    = pcloud_msg->header.stamp;
      pcloud2_msg.header.frame_id = pcloud_msg->header.frame_id;

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