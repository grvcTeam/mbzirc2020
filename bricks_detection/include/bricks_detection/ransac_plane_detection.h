/*!
 *      @file  ransac_plane_detection.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  24/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace mbzirc
{
class RANSACPlaneDetection
{
  public:
   RANSACPlaneDetection();
   virtual ~RANSACPlaneDetection(void);

   void detect(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>& plane_pcloud);

   pcl::SACSegmentation<pcl::PointXYZRGB> detector;

  private:
   bool checkPlane(const pcl::ModelCoefficients& coef);

   unsigned int _max_ransac_iterations;
};
}  // namespace mbzirc