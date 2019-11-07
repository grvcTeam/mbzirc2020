/*!
 *      @file  shape_detection.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  31/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <bricks_detection/types/image_item.h>

#include <bricks_detection/shape_detection.h>

namespace mbzirc
{
ShapeDetection::ShapeDetection() : _min_area(2.0), _poly_epsilon(3.0) {}

ShapeDetection::~ShapeDetection() {}

void ShapeDetection::detect(cv::Mat& binary_img, cv::Mat& color_img, const std::string color,
                            std::vector<ImageItem>& detected_items)
{
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;

   cv::Scalar bgr_color;
   if (color == "red")
      bgr_color = cv::Scalar(0, 0, 0);
   else if (color == "blue")
      bgr_color = cv::Scalar(255, 0, 0);
   else if (color == "green")
      bgr_color = cv::Scalar(0, 255, 0);
   else if (color == "orange")
      bgr_color = cv::Scalar(0, 128, 255);
   else
      bgr_color = cv::Scalar(0, 255, 255);

   cv::findContours(binary_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
   for (size_t i = 0; i < contours.size(); i++)
   {
      const double perimeter = cv::arcLength(contours[i], true);

      cv::Mat polygon;
      cv::approxPolyDP(cv::Mat(contours[i]), polygon, _poly_epsilon * perimeter, true);

      const double area = fabs(cv::contourArea(polygon));
      if (area < _min_area) continue;

      cv::Moments moments      = cv::moments(polygon);
      const cv::Point centroid = cv::Point(cvRound(moments.m10 / moments.m00), cvRound(moments.m01 / moments.m00));
      const double mu20_prime  = moments.mu20 / moments.m00;  // mu00 = m00
      const double mu02_prime  = moments.mu02 / moments.m00;  // mu00 = m00
      const double mu11_prime  = moments.mu11 / moments.m00;  // mu00 = m00
      const double theta       = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);

      ImageItem image_item(color);
      image_item.centroid    = centroid;
      image_item.area        = area;
      image_item.perimeter   = perimeter;
      image_item.orientation = theta;

      detected_items.push_back(image_item);

      cv::circle(color_img, centroid, 4, bgr_color, -1);
      cv::drawContours(color_img, contours, i, bgr_color);
   }
}

void ShapeDetection::setMinArea(const double& min_area) { _min_area = min_area; }
void ShapeDetection::setPolyEpsilon(const float& poly_epsilon) { _poly_epsilon = poly_epsilon; }

}  // namespace mbzirc