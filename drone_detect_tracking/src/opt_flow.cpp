/*!
 * @file opt_flow.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 26/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <drone_detector_tracking/opt_flow.h>
#include <drone_detector_tracking/opt_flow_points_set.h>
#include <iostream>

namespace mbzirc
{
OptFlow::OptFlow(cv::Mat& first_image, cv::TermCriteria& termcrit, int static_points_num, int dynamic_points_num,
                 float& velocity_tolerance)
{
   first_image.copyTo(_prev_gray_img);
   _termcrit                  = termcrit;
   _static_points_num         = static_points_num;
   _dynamic_points_num        = dynamic_points_num;
   _static_velocity_tolerance = velocity_tolerance;
   _subPixWinSize             = cv::Size(5, 5);
   _winSize                   = cv::Size(11, 11);
   _image_size                = cv::Size(first_image.cols, first_image.rows);
   _static_points             = new OptFlowPointsSet(_static_velocity_tolerance);
   _dynamic_points            = new OptFlowPointsSet(_static_velocity_tolerance);

   cv::Point zero(0, 0);
   _dynamic_points->addLocalizedRandomPoints(_dynamic_points_num, zero, _image_size);
}

OptFlow::~OptFlow() {}

void OptFlow::updateStaticPoints(cv::Mat& gray_image)
{
   if (_static_points->points().size() < _static_points_num)
   {
      cv::goodFeaturesToTrack(gray_image, _next_static_points, _static_points_num, 0.01,
                              _static_points_num - _static_points->points().size());

      cv::cornerSubPix(gray_image, _next_static_points, _subPixWinSize, cv::Size(-1, -1), _termcrit);

      _static_points->addNewPoints(_next_static_points);
   }

   std::vector<float> err;

   cv::calcOpticalFlowPyrLK(_prev_gray_img, gray_image, _static_points->points(), _next_static_points, _static_status,
                            err, _winSize, 3, _termcrit, 0, 0.001);

   _static_points->updatePoints(_next_static_points, _static_status);
   cv::swap(_prev_gray_img, gray_image);

   std::vector<bool> are_static = _static_points->checkVelHomogeneity();
   // TODO:  Tune velocity_threshold & add wrong points to moving objects

   /*------------------------------------------Debug----------------------------------------------------- */
   // for (auto is_static : are_static)
   // {
   //    std::cout << is_static << std::endl;
   // }
   // std::cout << std::endl;
}

void OptFlow::updateDynamicPoints(cv::Mat& gray_image)
{
   if (_dynamic_points->points().size() < _dynamic_points_num)
   {
      cv::Point zero(0, 0);
      _dynamic_points->addLocalizedRandomPoints(_dynamic_points_num - _dynamic_points->points().size(), zero,
                                                _image_size);
      // Create new random points

      // cv::goodFeaturesToTrack(gray_image, _next_dynamic_points, _dynamic_points_num, 0.01,
      //                         _dynamic_points_num - _dynamic_points->points().size());

      // cv::cornerSubPix(gray_image, _next_dynamic_points, _subPixWinSize, cv::Size(-1, -1), _termcrit);

      // _dynamic_points->addNewPoints(_next_dynamic_points);
   }

   std::vector<float> err;

   cv::calcOpticalFlowPyrLK(_prev_gray_img, gray_image, _dynamic_points->points(), _next_dynamic_points,
                            _dynamic_status, err, _winSize, 3, _termcrit, 0, 0.001);

   _dynamic_points->updatePoints(_next_dynamic_points, _dynamic_status);

   std::vector<bool> is_static = _dynamic_points->checkVelHomogeneity(_static_points->velocityAverage());

   for (size_t i = 0; i < _dynamic_points->points().size(); i++)
   {
      if (is_static[i])
      {
         _dynamic_points->points().erase(_dynamic_points->points().begin() + i);
      }
      else
      {
         std::cout << "Good dynamic point " << std::endl;
      }
   }
}

void OptFlow::drawStaticPoints(cv::Mat& color_image)
{
   size_t i;
   for (i = 0; i < _next_static_points.size(); i++)
   {
      if (_static_status[i])
      {
         circle(color_image, _static_points->points()[i], 3, cv::Scalar(255, 0, 0), -1, 8);
         circle(color_image, _next_static_points[i], 3, cv::Scalar(0, 255, 0), -1, 8);
      }
   }
}
void OptFlow::drawDynamicPoints(cv::Mat& color_image)
{
   size_t i;
   for (i = 0; i < _next_dynamic_points.size(); i++)
   {
      if (1)  //_dynamic_status[i]
      {
         circle(color_image, _dynamic_points->points()[i], 3, cv::Scalar(0, 0, 255), -1, 8);
         circle(color_image, _next_dynamic_points[i], 3, cv::Scalar(0, 255, 255), -1, 8);
      }
   }
}

}  // namespace mbzirc