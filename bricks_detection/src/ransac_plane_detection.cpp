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
   _coef0 = _coef1 = _coef2 = 0.2;
   _enabled                 = true;
}

RANSACPlaneDetection::~RANSACPlaneDetection() {}

void RANSACPlaneDetection::detect(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>& plane_pcloud)
{
   if (!_enabled || pcloud.empty())
   {
      plane_pcloud = pcloud;
      return;
   }

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

void RANSACPlaneDetection::toggle(const bool& enable) { _enabled = enable; }

void RANSACPlaneDetection::setMaxIterations(const int& max_iterations) { _max_ransac_iterations = max_iterations; }

void RANSACPlaneDetection::setMaxCoefs(const float& coef0, const float& coef1, const float& coef2)
{
   _coef0 = coef0;
   _coef1 = coef1;
   _coef2 = coef2;
}

bool RANSACPlaneDetection::checkPlane(const pcl::ModelCoefficients& coef)
{
   float constant = fmod(fabs((coef.values[4]) / (coef.values[3])), _coef2);
   if ((fabs(coef.values[0]) < _coef0) && (fabs(coef.values[1]) < _coef1) && (fabs(coef.values[3]) < 0.1))
      return false;
   else if (((fabs(coef.values[0]) < _coef0) && (fabs(coef.values[1]) < _coef1)) && (constant < _coef2))
      return true;
   return false;
}

}  // namespace mbzirc