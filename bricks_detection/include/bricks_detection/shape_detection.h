/*!
 *      @file  shape_detection.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  31/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace mbzirc
{
class ImageItem;
class ShapeDetection
{
  public:
   ShapeDetection();
   virtual ~ShapeDetection(void);

   void detect(cv::Mat& img, cv::Mat& color_img, const std::string color, std::vector<ImageItem>& detected_items);

   void setMinArea(const double& min_area);
   void setPolyEpsilon(const float& poly_epsilon);

  private:
   double _min_area;
   float _poly_epsilon;
};
}  // namespace mbzirc