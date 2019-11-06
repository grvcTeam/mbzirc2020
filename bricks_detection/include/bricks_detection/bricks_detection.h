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

#include <opencv2/core/core.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/TransformStamped.h>

#include <bricks_detection/filtering/color_filtering.h>
#include <bricks_detection/filtering/distance_filtering.h>
#include <bricks_detection/pose_estimation.h>
#include <bricks_detection/ransac_plane_detection.h>
#include <bricks_detection/shape_detection.h>

namespace mbzirc
{
class BricksDetection
{
  public:
   BricksDetection();
   virtual ~BricksDetection(void);

   void processData(cv::Mat& img, cv::Mat& filtered_img, std::vector<ImageItem>& detected_items);
   void processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                    geometry_msgs::TransformStamped& transform, std::vector<PCloudItem>& detected_items);

   ColorFiltering* color_filtering;
   DistanceFiltering* distance_filtering;
   RANSACPlaneDetection* plane_detector;
   ShapeDetection* shape_detector;
   PoseEstimation* pose_estimation;

  private:
   void filtering(cv::Mat& img, std::map<std::string, cv::Mat>& color_imgs_cluster);
   void filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster);

   void findRectangles(cv::Mat& img, std::map<std::string, cv::Mat>& color_imgs_cluster,
                       std::vector<ImageItem>& detected_items);

   void transform(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                  geometry_msgs::TransformStamped& transform);

   void planeSegmentation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster);

   void poseEstimation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                       std::vector<PCloudItem>& detected_items);
};
}  // namespace mbzirc