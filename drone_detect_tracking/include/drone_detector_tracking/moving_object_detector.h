/*!
 * @file moving_object_detector.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 10/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <opencv2/core/core.hpp>

namespace mbzirc
{
class MovingObjectParameters;
class IntrinsicParameters;
class OptFlow;

class MovingObjectDetector
{
  public:
   MovingObjectDetector(MovingObjectParameters* params);
   ~MovingObjectDetector();
   cv::Mat processFrame(cv::Mat& rgb_img);
   void setParams(MovingObjectParameters* params);
   void setCameraIntrinsics(const IntrinsicParameters& params);
   void updateLK(cv::Mat& rgb_img);

  private:
   cv::Mat _raw_depth_img;
   MovingObjectParameters* _params;

   OptFlow* _opt_flow = {nullptr};

   cv::TermCriteria _termcrit;

   int _static_points_num, _dynamic_points_num;
};

}  // namespace mbzirc