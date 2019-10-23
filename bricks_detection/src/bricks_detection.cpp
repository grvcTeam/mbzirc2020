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

#include <bricks_detection/bricks_detection.h>

namespace mbzirc
{
BricksDetection::BricksDetection()
{
   color_filtering    = new ColorFiltering();
   distance_filtering = new DistanceFiltering();
}

BricksDetection::~BricksDetection() {}

void BricksDetection::processData(pcl::PointCloud<pcl::PointXYZRGB>& pcloud) { filtering(pcloud); }

void BricksDetection::filtering(pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{
   color_filtering->pointcloudFilter(pcloud);
   distance_filtering->pointcloudFilter(pcloud);
}
}  // namespace mbzirc