/*!
 *      @file  pose_estimation.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  06/11/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <pcl/features/moment_of_inertia_estimation.h>

namespace mbzirc
{
class PCloudItem;
class PoseEstimation
{
  public:
   PoseEstimation();
   virtual ~PoseEstimation(void);

   bool estimate(pcl::PointCloud<pcl::PointXYZRGB>& pcloud, PCloudItem& pcloud_item);

  private:
   pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> _feature_extractor;
};
}  // namespace mbzirc