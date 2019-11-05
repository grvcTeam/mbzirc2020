/*!
 * @file opt_flow_points_set.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 25/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */
#include <drone_detector_tracking/opt_flow_points_set.h>
#include <iostream>

namespace mbzirc
{
OptFlowPointsSet::OptFlowPointsSet(float& tolerance) { _tolerance = tolerance; }

OptFlowPointsSet::OptFlowPointsSet(std::vector<cv::Point2f>& points) { _points = points; }

OptFlowPointsSet::~OptFlowPointsSet() {}

std::vector<cv::Point2f>& OptFlowPointsSet::points() { return _points; }
void OptFlowPointsSet::points(const std::vector<cv::Point2f>& points) { _points = points; }

cv::Point2f& OptFlowPointsSet::velocityAverage() { return _velocity_average; }

float& OptFlowPointsSet::tolerance() { return _tolerance; }

void OptFlowPointsSet::updatePoints(std::vector<cv::Point2f>& new_points, std::vector<uchar>& status)
{
   _velocity_average.x = _velocity_average.y = .0;
   size_t i, k;
   for (i = k = 0; i < _points.size(); i++)
   {
      if (status[i])
      {
         _velocities[k] = new_points[i] - _points[i];
         _points[k++]   = new_points[i];
         _velocity_average += _velocities[i];
      }
   }
   _velocities.resize(k);
   _points.resize(k);
   _velocity_average /= (float)++k;
}

void OptFlowPointsSet::addNewPoints(std::vector<cv::Point2f>& new_points)
{
   for (auto point : new_points)
   {
      _points.push_back(point);
      _velocities.push_back(cv::Point2f(0, 0));
   }
}

void OptFlowPointsSet::addLocalizedRandomPoints(const size_t& new_rand_points, cv::Point& tr_starting_point,
                                                cv::Size& roi_dimensions)
{
   for (size_t i = 0; i < new_rand_points; i++)
   {
      _points.emplace_back(cv::Point2f(rand() % roi_dimensions.width + tr_starting_point.x,
                                       rand() % roi_dimensions.height + tr_starting_point.y));

      _velocities.emplace_back(cv::Point2f(0, 0));
   }
}

std::vector<bool> OptFlowPointsSet::checkVelHomogeneity()
{
   std::vector<bool> homogeneous;
   for (auto velocity : _velocities)
   {
      if (velocity.x > _velocity_average.x + _tolerance || velocity.x < -_velocity_average.x - _tolerance)
      {
         homogeneous.push_back(false);
         continue;
      }
      if (velocity.y > _velocity_average.y + _tolerance || velocity.y < -_velocity_average.y - _tolerance)
      {
         homogeneous.push_back(false);
         continue;
      }
      homogeneous.push_back(true);
   }
   return homogeneous;
}

std::vector<bool> OptFlowPointsSet::checkVelHomogeneity(cv::Point2f velocity_average)
{
   std::vector<bool> homogeneous;
   for (auto velocity : _velocities)
   {
      if (velocity.x > velocity_average.x + _tolerance || velocity.x < -velocity_average.x - _tolerance)
      {
         homogeneous.push_back(false);
         continue;
      }
      if (velocity.y > velocity_average.y + _tolerance || velocity.y < -velocity_average.y - _tolerance)
      {
         homogeneous.push_back(false);
         continue;
      }
      homogeneous.push_back(true);
   }
   return homogeneous;
}

}  // namespace mbzirc