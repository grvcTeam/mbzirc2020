/*!
 *      @file  color_filtering.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bricks_detection/types/hsv.h>

namespace mbzirc
{
class ColorFiltering
{
  public:
   ColorFiltering();
   virtual ~ColorFiltering(void);

   void addHSVFilter(const std::string& json_path);
   void addHSVFilter(const HSV& lower_hsv, const HSV& upper_hsv);

   void pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud);

  private:
   bool inRange(const float& vmin, const float& vmax, const float& value) const;

   std::vector<std::pair<HSV, HSV>> _hsv_filters;
};
}  // namespace mbzirc