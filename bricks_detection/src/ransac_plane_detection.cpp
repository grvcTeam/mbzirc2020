/*!
 *      @file  ransac_plane_detection.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  24/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <pcl/filters/extract_indices.h>

#include <bricks_detection/ransac_plane_detection.h>

namespace mbzirc
{
RANSACPlaneDetection::RANSACPlaneDetection()
{
   detector.setOptimizeCoefficients(true);
   detector.setModelType(pcl::SACMODEL_PLANE);
   detector.setMethodType(pcl::SAC_RANSAC);
   detector.setDistanceThreshold(0.01);

   _max_ransac_iterations = 3;
}

RANSACPlaneDetection::~RANSACPlaneDetection() {}

void RANSACPlaneDetection::detect(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>& plane_pcloud)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcloud);

   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
   pcl::ExtractIndices<pcl::PointXYZRGB> extractor;

   for (unsigned int i = 0; i < _max_ransac_iterations; i++)
   {
      extractor.setNegative(true);
      detector.setInputCloud(p_pcloud);
      detector.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) return;  // no plane inside

      bool checked_plane = checkPlane(*coefficients);

      if (checked_plane)
      {
         extractor.setNegative(false);

         extractor.setInputCloud(p_pcloud);
         extractor.setIndices(inliers);
         extractor.filter(plane_pcloud);
         return;
      }
   }
}

bool RANSACPlaneDetection::checkPlane(const pcl::ModelCoefficients& coef)
{
   float constant = fmod(fabs((coef.values[4]) / (coef.values[3])), 0.2);
   if (((fabs(coef.values[0]) < 0.2) && (fabs(coef.values[1]) < 0.2)) && (constant < 0.2)) return true;
   return false;
}

}  // namespace mbzirc