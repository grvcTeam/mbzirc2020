/*!
 *      @file  bricks_detection.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  16/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <pcl_ros/transforms.h>

#include <bricks_detection/bricks_detection.h>

namespace mbzirc
{
BricksDetection::BricksDetection()
{
   color_filtering    = new ColorFiltering();
   distance_filtering = new DistanceFiltering();
   plane_detector     = new RANSACPlaneDetection();
}

BricksDetection::~BricksDetection() {}

void BricksDetection::processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                  tf::StampedTransform& transform)
{
   if (pcloud.empty()) return;

   this->filtering(pcloud, color_pcloud_cluster);
   this->transform(color_pcloud_cluster, transform);
   // this->planeSegmentation(color_pcloud_cluster);
}

void BricksDetection::filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster)
{
   color_filtering->pointcloudFilter(pcloud, color_pcloud_cluster);

   for (auto color_pcloud : color_pcloud_cluster)
   {
      distance_filtering->pointcloudFilter(color_pcloud.second);
      color_pcloud_cluster[color_pcloud.first] = color_pcloud.second;
   }
}

void BricksDetection::transform(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                tf::StampedTransform& transform)
{
   for (auto color_pcloud : color_pcloud_cluster)
   {
      pcl::PointCloud<pcl::PointXYZRGB> tf_pcloud;
      pcl_ros::transformPointCloud(color_pcloud.second, tf_pcloud, transform);

      color_pcloud_cluster[color_pcloud.first] = tf_pcloud;
   }
}

void BricksDetection::planeSegmentation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster)
{
   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> result_color_pcloud_cluster;

   for (auto color_pcloud : color_pcloud_cluster)
   {
      pcl::PointCloud<pcl::PointXYZRGB> plane_pcloud;
      plane_detector->detect(color_pcloud.second, plane_pcloud);

      if (!plane_pcloud.empty())
      {
         result_color_pcloud_cluster[color_pcloud.first] = plane_pcloud;
      }
   }

   color_pcloud_cluster = result_color_pcloud_cluster;
}
}  // namespace mbzirc