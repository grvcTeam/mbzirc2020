/*!
 *      @file  pose_estimation.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  06/11/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <boost/make_shared.hpp>

#include <pcl/common/centroid.h>

#include <bricks_detection/pose_estimation.h>
#include <bricks_detection/types/pcloud_item.h>

namespace mbzirc
{
PoseEstimation::PoseEstimation() {}
PoseEstimation::~PoseEstimation() {}

bool PoseEstimation::estimate(pcl::PointCloud<pcl::PointXYZRGB>& pcloud, PCloudItem& pcloud_item)
{
   Eigen::Vector4f centroid;
   if (!pcl::compute3DCentroid(pcloud, centroid)) return false;

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcloud);

   _feature_extractor.setInputCloud(p_pcloud);
   _feature_extractor.compute();

   Eigen::Matrix3f rotational_matrix_OBB;
   pcl::PointXYZRGB min_point_OBB;
   pcl::PointXYZRGB max_point_OBB;
   pcl::PointXYZRGB position_OBB;

   if (!_feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB)) return false;

   pcloud_item.position    = centroid;
   pcloud_item.orientation = Eigen::Quaternionf(rotational_matrix_OBB);
   if (pcloud_item.orientation.y() < 0) pcloud_item.orientation.y() = -pcloud_item.orientation.y();

   return true;
}
}  // namespace mbzirc