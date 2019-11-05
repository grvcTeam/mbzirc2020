/*!
 * @file opt_flow_points_set.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 25/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */
#pragma once

#include <opencv2/core/core.hpp>

namespace mbzirc
{
class OptFlowPointsSet
{
  public:
   OptFlowPointsSet(float& tolerance);

   OptFlowPointsSet(std::vector<cv::Point2f>& points);

   virtual ~OptFlowPointsSet();

   std::vector<cv::Point2f>& points();
   void points(const std::vector<cv::Point2f>& points);

   cv::Point2f& velocityAverage();

   float& tolerance();

   void updatePoints(std::vector<cv::Point2f>& new_points, std::vector<uchar>& status);
   void addNewPoints(std::vector<cv::Point2f>& new_points);
   void addLocalizedRandomPoints(const size_t& new_rand_points, cv::Point& tr_starting_point, cv::Size& roi_dimensions);
   std::vector<bool> checkVelHomogeneity();
   std::vector<bool> checkVelHomogeneity(cv::Point2f velocity_average);

  private:
   std::vector<cv::Point2f> _points;
   std::vector<cv::Point2f> _velocities;

   float _tolerance;

   cv::Point2f _velocity_average;
};

}  // namespace mbzirc