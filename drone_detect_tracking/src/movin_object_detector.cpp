/*!
 * @file moving_object_detector.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 29/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <math.h>

#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"

#include <drone_detector_tracking/moving_object_detector.h>
#include <drone_detector_tracking/moving_object_parameters.h>
#include <drone_detector_tracking/opt_flow.h>
#include <drone_detector_tracking/opt_flow_points_set.h>

namespace mbzirc
{
MovingObjectDetector::MovingObjectDetector(MovingObjectParameters* params)
{
   setParams(params);
   _termcrit = cv::TermCriteria(_params->lk_criteria_type, _params->lk_max_count, _params->lk_epsilon);
}

MovingObjectDetector::~MovingObjectDetector() {}

cv::Mat MovingObjectDetector::processFrame(cv::Mat& rgb_img)
{
   _raw_depth_img = rgb_img.clone();

   return rgb_img;
}

void MovingObjectDetector::setParams(MovingObjectParameters* params)
{
   _params   = params;
   _termcrit = cv::TermCriteria(_params->lk_criteria_type, _params->lk_max_count, _params->lk_epsilon);
}

void MovingObjectDetector::setCameraIntrinsics(const IntrinsicParameters& params) {}

void MovingObjectDetector::updateLK(cv::Mat& rgb_img)
{
   cv::Mat gray_img;
   cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);

   if (_opt_flow == nullptr)
   {
      // TODO: parametrice number of static and dynamic points
      _opt_flow = new OptFlow(gray_img, _termcrit, _params->static_points_num, _params->dynamic_points_num,
                              _params->velocity_tolerance);
      return;
   }
   _opt_flow->updateStaticPoints(gray_img);
   _opt_flow->updateDynamicPoints(gray_img);
   _opt_flow->drawStaticPoints(rgb_img);
   _opt_flow->drawDynamicPoints(rgb_img);
}

}  // namespace mbzirc