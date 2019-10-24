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
                                  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster)
{
   if (pcloud.empty()) return;

   filtering(pcloud, pcloud_color_cluster);
   // planeSegmentation(pcloud_color_cluster);
}

void BricksDetection::filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster)
{
   color_filtering->pointcloudFilter(pcloud, pcloud_color_cluster);

   for (auto color_pcloud : pcloud_color_cluster)
   {
      distance_filtering->pointcloudFilter(color_pcloud.second);
      pcloud_color_cluster[color_pcloud.first] = color_pcloud.second;
   }
}

void BricksDetection::planeSegmentation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster)
{
   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> result_pcloud_color_cluster;

   for (auto color_pcloud : pcloud_color_cluster)
   {
      pcl::PointCloud<pcl::PointXYZRGB> plane_pcloud;
      plane_detector->detect(color_pcloud.second, plane_pcloud);

      if (!plane_pcloud.empty())
      {
         result_pcloud_color_cluster[color_pcloud.first] = plane_pcloud;
      }
   }

   pcloud_color_cluster = result_pcloud_color_cluster;
}
}  // namespace mbzirc