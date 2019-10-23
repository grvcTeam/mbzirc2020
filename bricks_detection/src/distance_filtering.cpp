/*!
 *      @file  distance_filtering.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  22/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <boost/make_shared.hpp>

#include <bricks_detection/filtering/distance_filtering.h>

namespace mbzirc
{
DistanceFiltering::DistanceFiltering() {}

DistanceFiltering::~DistanceFiltering() {}

void DistanceFiltering::pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcloud);

   if (_enable_voxel_grid)
   {
      voxel_grid.setInputCloud(p_pcloud);
      voxel_grid.filter(pcloud);
   }

   if (_enable_z_filtering)
   {
      z_filtering.setInputCloud(p_pcloud);
      z_filtering.setFilterFieldName("z");
      z_filtering.filter(pcloud);
   }

   if (_enable_sor_filtering)
   {
      sor_filtering.setInputCloud(p_pcloud);
      sor_filtering.filter(pcloud);
   }
}

void DistanceFiltering::toggleFilters(const bool voxel, const bool z_axis, const bool sor)
{
   _enable_voxel_grid    = voxel;
   _enable_z_filtering   = z_axis;
   _enable_sor_filtering = sor;
}

}  // namespace mbzirc