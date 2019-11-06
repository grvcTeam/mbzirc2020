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

#include <bricks_detection/types/image_item.h>
#include <bricks_detection/types/pcloud_item.h>

#include <bricks_detection/bricks_detection.h>

namespace mbzirc
{
BricksDetection::BricksDetection()
{
   color_filtering    = new ColorFiltering();
   distance_filtering = new DistanceFiltering();
   plane_detector     = new RANSACPlaneDetection();
   shape_detector     = new ShapeDetection();
   pose_estimation    = new PoseEstimation();
}

BricksDetection::~BricksDetection() {}

void BricksDetection::processData(cv::Mat& img, cv::Mat& filtered_img, std::vector<ImageItem>& detected_items)
{
   if (img.empty()) return;

   std::map<std::string, cv::Mat> color_imgs_cluster;

   this->filtering(img, color_imgs_cluster);
   this->findRectangles(img, color_imgs_cluster, detected_items);

   filtered_img = img;
}

void BricksDetection::processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                  geometry_msgs::TransformStamped& transform, std::vector<PCloudItem>& detected_items)
{
   if (pcloud.empty()) return;

   this->filtering(pcloud, color_pcloud_cluster);
   this->transform(color_pcloud_cluster, transform);
   this->planeSegmentation(color_pcloud_cluster);
   this->poseEstimation(color_pcloud_cluster, detected_items);
}

void BricksDetection::filtering(cv::Mat& img, std::map<std::string, cv::Mat>& color_imgs_cluster)
{
   color_filtering->imageFilter(img, color_imgs_cluster);
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

void BricksDetection::findRectangles(cv::Mat& img, std::map<std::string, cv::Mat>& color_imgs_cluster,
                                     std::vector<ImageItem>& detected_items)
{
   for (auto color_mask : color_imgs_cluster)
   {
      shape_detector->detect(color_mask.second, img, color_mask.first, detected_items);
   }
}

void BricksDetection::transform(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                geometry_msgs::TransformStamped& transform)
{
   for (auto color_pcloud : color_pcloud_cluster)
   {
      pcl::PointCloud<pcl::PointXYZRGB> tf_pcloud;
      pcl_ros::transformPointCloud(color_pcloud.second, tf_pcloud, transform.transform);

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

void BricksDetection::poseEstimation(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster,
                                     std::vector<PCloudItem>& detected_items)
{
   for (auto color_pcloud : color_pcloud_cluster)
   {
      PCloudItem item(color_pcloud.first);
      if (pose_estimation->estimate(color_pcloud.second, item)) detected_items.push_back(item);
   }
}

}  // namespace mbzirc