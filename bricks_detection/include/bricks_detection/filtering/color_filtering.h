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

#include <set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bricks_detection/types/hsv_range.h>

namespace mbzirc
{
class ColorFiltering
{
  public:
   ColorFiltering();
   virtual ~ColorFiltering(void);

   void addHSVFilter(const std::string& json_path);
   void addHSVFilter(const HSV& lower_hsv, const HSV& upper_hsv, const std::string& color_name);
   void setMinPointsPerColor(const int& min);

   void pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                         std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster);

  private:
   bool inRange(const float& vmin, const float& vmax, const float& value) const;

   std::vector<HSVRange> _hsv_filters;
   std::set<std::string> _colors;

   unsigned int _min_cluster_size;
};
}  // namespace mbzirc