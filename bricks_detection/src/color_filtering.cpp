/*!
 *      @file  color_filtering.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <limits>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <bricks_detection/filtering/color_filtering.h>

namespace mbzirc
{
ColorFiltering::ColorFiltering() {}

ColorFiltering::~ColorFiltering() {}

void ColorFiltering::addHSVFilter(const std::string& json_path)
{
   namespace pt = boost::property_tree;

   pt::ptree root;
   pt::read_json(json_path, root);

   _hsv_filters.clear();

   for (pt::ptree::value_type& color : root.get_child("colors"))
   {
      pt::ptree color_root = color.second;

      const float h_min = color_root.get<float>("min.h");
      const float s_min = color_root.get<float>("min.s");
      const float v_min = color_root.get<float>("min.v");

      const float h_max = color_root.get<float>("max.h");
      const float s_max = color_root.get<float>("max.s");
      const float v_max = color_root.get<float>("max.v");

      const HSV hsv_min(h_min, s_min, v_min);
      const HSV hsv_max(h_max, s_max, v_max);
      addHSVFilter(hsv_min, hsv_max);
   }
}

void ColorFiltering::addHSVFilter(const HSV& lower_hsv, const HSV& upper_hsv)
{
   _hsv_filters.push_back(std::make_pair(lower_hsv, upper_hsv));
}

void ColorFiltering::pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{
   pcl::PointCloud<pcl::PointXYZRGB> pcloud_filtered;
   pcloud_filtered.height = 1;

   pcl::PointCloud<pcl::PointXYZHSV> pcloud_hsv;
   pcl::PointCloudXYZRGBtoXYZHSV(pcloud, pcloud_hsv);

   for (size_t i = 0; i < pcloud_hsv.width*pcloud_hsv.height; i++)
   {
      for (auto hsv_filter : _hsv_filters)
      {
         if (!inRange(hsv_filter.first.h, hsv_filter.second.h, pcloud_hsv.points[i].h)) continue;
         if (!inRange(hsv_filter.first.s, hsv_filter.second.s, pcloud_hsv.points[i].s)) continue;
         if (!inRange(hsv_filter.first.v, hsv_filter.second.v, pcloud_hsv.points[i].v)) continue;

         pcloud_filtered.points.push_back(pcloud.points[i]);
         pcloud_filtered.width++;
      }
   }

   pcloud = pcloud_filtered;
}

bool ColorFiltering::inRange(const float& vmin, const float& vmax, const float& value) const
{
   return (value >= vmin) && (value <= vmax);
}

}  // namespace mbzirc
