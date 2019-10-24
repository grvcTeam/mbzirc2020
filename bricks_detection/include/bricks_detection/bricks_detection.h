/*!
 *      @file  bricks_detection.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  16/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_datatypes.h>

#include <bricks_detection/filtering/color_filtering.h>
#include <bricks_detection/filtering/distance_filtering.h>
#include <bricks_detection/ransac_plane_detection.h>

namespace mbzirc
{
class BricksDetection
{
  public:
   BricksDetection();
   virtual ~BricksDetection(void);

   void processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster,
                    tf::StampedTransform& transform);

   ColorFiltering* color_filtering;
   DistanceFiltering* distance_filtering;
   RANSACPlaneDetection* plane_detector;

  private:
   void filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster);

   void planeSegmentation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster);
};
}  // namespace mbzirc