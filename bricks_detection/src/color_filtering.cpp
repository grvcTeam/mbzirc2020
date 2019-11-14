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
#include <map>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <bricks_detection/filtering/color_filtering.h>

namespace mbzirc
{
ColorFiltering::ColorFiltering() { _min_cluster_size = 0; }

ColorFiltering::~ColorFiltering() {}

void ColorFiltering::addHSVFilter(const std::string& json_path)
{
   namespace pt = boost::property_tree;

   pt::ptree root;
   pt::read_json(json_path, root);

   _hsv_filters.clear();
   _colors.clear();

   for (pt::ptree::value_type& color : root.get_child("colors"))
   {
      pt::ptree color_root = color.second;

      const float h_min = color_root.get<float>("min.h");
      const float s_min = color_root.get<float>("min.s");
      const float v_min = color_root.get<float>("min.v");

      const float h_max = color_root.get<float>("max.h");
      const float s_max = color_root.get<float>("max.s");
      const float v_max = color_root.get<float>("max.v");

      const std::string color_name = color_root.get<std::string>("name");

      const HSV hsv_min(h_min, s_min, v_min);
      const HSV hsv_max(h_max, s_max, v_max);
      addHSVFilter(hsv_min, hsv_max, color_name);
   }
}

void ColorFiltering::addHSVFilter(const HSV& lower_hsv, const HSV& upper_hsv, const std::string& color_name)
{
   _hsv_filters.push_back(HSVRange(lower_hsv, upper_hsv, color_name));
   _colors.insert(color_name);
}

void ColorFiltering::setMinPointsPerColor(const int& min) { _min_cluster_size = min; }

void ColorFiltering::imageFilter(cv::Mat& img, std::map<std::string, cv::Mat>& color_imgs_cluster)
{
   CV_Assert(img.depth() == CV_8U);

   for (auto color : _colors)
   {
      color_imgs_cluster[color] = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
   }

   cv::Mat hsv_img;
   cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

   unsigned char* p_hsv_data = (unsigned char*)(hsv_img.data);
   for (int i = 0; i < hsv_img.rows; i++)
   {
      for (int j = 0; j < hsv_img.cols; j++)
      {
         int ocv_h = p_hsv_data[hsv_img.step * i + hsv_img.channels() * j];
         int ocv_s = p_hsv_data[hsv_img.step * i + hsv_img.channels() * j + 1];
         int ocv_v = p_hsv_data[hsv_img.step * i + hsv_img.channels() * j + 2];

         const float h = ocv_h * 2.0f;    // [0, 180] to [0, 360]
         const float s = ocv_s / 255.0f;  // [0, 255] to [0, 1]
         const float v = ocv_v / 255.0f;  // [0, 255] to [0, 1]

         for (auto hsv_filter : _hsv_filters)
         {
            if (!inRange(hsv_filter.min.h, hsv_filter.max.h, h)) continue;
            if (!inRange(hsv_filter.min.s, hsv_filter.max.s, s)) continue;
            if (!inRange(hsv_filter.min.v, hsv_filter.max.v, v)) continue;

            color_imgs_cluster[hsv_filter.color_name].at<uchar>(i, j) = 255;
         }
      }
   }
}

void ColorFiltering::pointcloudFilter(pcl::PointCloud<pcl::PointXYZRGB>& pcloud,
                                      std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>>& color_pcloud_cluster)
{
   for (auto color : _colors)
   {
      color_pcloud_cluster[color].height = 1;
   }

   pcl::PointCloud<pcl::PointXYZHSV> pcloud_hsv;
   pcl::PointCloudXYZRGBtoXYZHSV(pcloud, pcloud_hsv);

   for (size_t i = 0; i < pcloud_hsv.width * pcloud_hsv.height; i++)
   {
      for (auto hsv_filter : _hsv_filters)
      {
         if (!inRange(hsv_filter.min.h, hsv_filter.max.h, pcloud_hsv.points[i].h)) continue;
         if (!inRange(hsv_filter.min.s, hsv_filter.max.s, pcloud_hsv.points[i].s)) continue;
         if (!inRange(hsv_filter.min.v, hsv_filter.max.v, pcloud_hsv.points[i].v)) continue;

         color_pcloud_cluster[hsv_filter.color_name].points.push_back(pcloud.points[i]);
         color_pcloud_cluster[hsv_filter.color_name].width++;
      }
   }

   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> result_color_pcloud_cluster;
   for (auto color_pcloud : color_pcloud_cluster)
   {
      if (color_pcloud.second.size() >= _min_cluster_size)
      {
         result_color_pcloud_cluster[color_pcloud.first] = color_pcloud.second;
      }
   }

   color_pcloud_cluster = result_color_pcloud_cluster;
}

bool ColorFiltering::inRange(const float& vmin, const float& vmax, const float& value) const
{
   return (value >= vmin) && (value <= vmax);
}

}  // namespace mbzirc