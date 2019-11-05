/*!
 * @file opt_flow.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 26/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"

namespace mbzirc
{
class OptFlowPointsSet;

class OptFlow
{
  public:
   OptFlow(cv::Mat& first_image, cv::TermCriteria& termcrit, int static_points_num, int dynamic_points_num,
           float& velocity_tolerance);

   virtual ~OptFlow();

   void updateStaticPoints(cv::Mat& gray_image);
   void updateDynamicPoints(cv::Mat& gray_image);
   void drawStaticPoints(cv::Mat& color_image);
   void drawDynamicPoints(cv::Mat& color_image);

  private:
   cv::TermCriteria _termcrit;
   cv::Mat _prev_gray_img;

   int _static_points_num, _dynamic_points_num;

   float _static_velocity_tolerance;
   cv::Size _subPixWinSize, _winSize, _image_size;

   OptFlowPointsSet* _static_points;
   OptFlowPointsSet* _dynamic_points;

   std::vector<uchar> _static_status;
   std::vector<uchar> _dynamic_status;

   std::vector<cv::Point2f> _next_static_points;
   std::vector<cv::Point2f> _next_dynamic_points;
};

}  // namespace mbzirc