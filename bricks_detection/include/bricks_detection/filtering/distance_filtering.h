/*!
 *      @file  distance_filtering.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  22/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <bricks_detection/types/hsv.h>

namespace mbzirc
{
class DistanceFiltering
{
  public:
   DistanceFiltering();
   virtual ~DistanceFiltering(void);

   void pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud);

   void toggleFilters(const bool voxel, const bool z_axis, const bool sor);

   pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
   pcl::PassThrough<pcl::PointXYZRGB> z_filtering;
   pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filtering;

  private:
   bool _enable_voxel_grid;
   bool _enable_z_filtering;
   bool _enable_sor_filtering;
};
}  // namespace mbzirc