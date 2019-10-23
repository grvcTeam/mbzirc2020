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

#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>

#include <bricks_detection/bricks_detection.h>

namespace mbzirc
{
BricksDetection::BricksDetection()
{
   color_filtering    = new ColorFiltering();
   distance_filtering = new DistanceFiltering();
}

BricksDetection::~BricksDetection() {}

void BricksDetection::processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{
   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> pcloud_color_cluster;

   filtering(pcloud, pcloud_color_cluster);
   // planeSegmentation(pcloud);
}

void BricksDetection::filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& pcloud_color_cluster)
{
   if (pcloud.empty()) return;

   color_filtering->pointcloudFilter(pcloud, pcloud_color_cluster);

   for (auto color_pcloud : pcloud_color_cluster)
   {
      distance_filtering->pointcloudFilter(color_pcloud.second);
   }
}

void BricksDetection::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{
   if (pcloud.empty()) return;

   // TODO: move this to external class
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcloud);

   pcl::PointCloud<pcl::Normal>::Ptr p_cloud_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
   pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
   normal_estimator.setSearchMethod(tree);
   normal_estimator.setRadiusSearch(0.02);  // TODO: dynamic param

   normal_estimator.setInputCloud(p_pcloud);
   normal_estimator.compute(*p_cloud_normals);
}
}  // namespace mbzirc