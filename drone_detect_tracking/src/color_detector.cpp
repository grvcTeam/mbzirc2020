/*!
 * @file drone_detector.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 11/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <math.h>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>
#include <drone_detector_tracking/color_detector.h>
#include <drone_detector_tracking/georef.h>
#include <drone_detector_tracking/processed_image.h>
#include <drone_detector_tracking/reconfigure_parameters.h>
#include <drone_detector_tracking/valid_detection.h>

namespace mbzirc
{
ColorDetector::ColorDetector(ColorReconfigureParameters* params, const bool print_debug)
{
   _debug         = print_debug;
   _camera_params = new CameraParameters();
   setParams(params);
}

ColorDetector::~ColorDetector() {}

void ColorDetector::processFrame(cv::Mat& img, cv::Mat& dbg1, cv::Mat& dbg2, cv::Mat& dbg3, cv::Mat& dbg4)
{
   cv::Mat gray_img;

   // dbg3 = img.clone();
   cv::Mat b_mask, b_mask_proc;
   imageProcessing(img, b_mask, b_mask_proc);

   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;

   cv::findContours(b_mask_proc, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
   std::vector<cv::Rect> boundRect(contours.size());
   std::vector<std::vector<cv::Point>> contours_poly(contours.size());

   cv::cvtColor(b_mask_proc, b_mask_proc, cv::COLOR_GRAY2RGB);
   // std::cout << contours.size() << std::endl;

   _valid_detections.clear();

   cv::Mat roi;

   for (int i = 0; i < contours.size(); i++)
   {
      cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
      boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

      const float area = cv::contourArea(contours[i]);  // px

      const float circularity = getCircularity(area, contours[i]);

      if (circularity < _params->min_circularity || circularity > _params->max_circularity)
      {
         continue;
      }

      float x_depth = _params->ball_diameter * _camera_params->intrinsics.fx / boundRect[i].width;
      float y_depth = _params->ball_diameter * _camera_params->intrinsics.fy / boundRect[i].height;
      float depth   = sqrt(std::pow(x_depth, 2) + std::pow(y_depth, 2));

      if (depth < 0.1 || depth > 100)
      {
         continue;
      }

      cv::putText(b_mask_proc, "Depth: " + std::to_string(depth), cv::Point(boundRect[i].x - 20, boundRect[i].y - 30),
                  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(100, 100, 0), 2);

      cv::Scalar color = cv::Scalar(100, 100, 0);
      cv::drawContours(b_mask_proc, contours, i, color, 5, 8, hierarchy, 0, cv::Point());

      cv::Rect rect(std::max(0, (int)(boundRect[i].x - 10)), std::max(0, (int)(boundRect[i].y - 10)),
                    std::min(img.cols - (int)(boundRect[i].x - 10), (int)(boundRect[i].width + 20)),
                    std::min(img.rows - (int)(boundRect[i].y - 10), (int)(boundRect[i].height + 20)));

      roi = img(rect);

      cv::Mat roi_b_mask;
      roiProcessing(roi, roi_b_mask);

      std::vector<std::vector<cv::Point>> roi_contours;
      std::vector<cv::Vec4i> roi_hierarchy;

      cv::findContours(roi_b_mask, roi_contours, roi_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      if (roi_contours.size() == 1)
      {
         // Calculate Moments
         cv::Moments moments = cv::moments(roi_b_mask, false);

         // std::cout << boundRect[i].width << std::endl;
         // std::cout << boundRect[i].height << std::endl;

         cv::approxPolyDP(cv::Mat(roi_contours[0]), contours_poly[i], 1, true);
         boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));

         const float roi_cont_area = cv::contourArea(contours[i]);  // px

         // const float circularity2 = getCircularity(roi_cont_area, roi_contours[0]);
         // std::cout << boundRect[i].height << std::endl << std::endl << std::endl;
         // std::cout << roi_cont_area << std::endl;
         // std::cout << circularity2 << std::endl << std::endl;

         double fake_diameter = 4 * sqrt(roi_cont_area / M_PI);  // Mysterious 4 instead of 2

         float x_depth = _params->ball_diameter * _camera_params->intrinsics.fx / fake_diameter;
         float y_depth = _params->ball_diameter * _camera_params->intrinsics.fy / fake_diameter;
         float depth   = sqrt(std::pow(x_depth, 2) + std::pow(y_depth, 2));

         float sigma_xy =
             std::abs(_camera_params->intrinsics.cx - boundRect[i].x + (boundRect[i].width / 2.)) / (float)img.cols +
             std::abs(_camera_params->intrinsics.cy - boundRect[i].y + (boundRect[i].height / 2.)) / img.rows;

         ValidDetection valid_detection(rect.x + boundRect[i].x + (boundRect[i].width / 2.),
                                        rect.y + boundRect[i].y + (boundRect[i].height / 2.), depth, sigma_xy, 50);

         _valid_detections.push_back(valid_detection);
         cv::putText(b_mask_proc, "Depth: " + std::to_string(depth),
                     cv::Point(boundRect[i].x - 20, boundRect[i].y - 30), cv::FONT_HERSHEY_PLAIN, 2,
                     cv::Scalar(0, 255, 0), 2);

         cv::Scalar color = cv::Scalar(0, 255, 0);
         cv::drawContours(b_mask_proc, contours, i, color, 5, 8, hierarchy, 0, cv::Point());

         // cv::Rect rect(std::max(0, (int)(boundRect[i].x - 10)), std::max(0, (int)(boundRect[i].y - 10)),
         //               std::min(img.cols - (int)(boundRect[i].x - 10), (int)(boundRect[i].width + 20)),
         //               std::min(img.rows - (int)(boundRect[i].y - 10), (int)(boundRect[i].height + 20)));
      }
   }
   geolocaliceValidDetections();

   dbg2 = b_mask_proc;
   dbg1 = roi;
   dbg3 = b_mask;

   return;
}

void ColorDetector::imageProcessing(cv::Mat& img, cv::Mat& b_mask, cv::Mat& b_mask_proc)
{
   // cv::GaussianBlur(img, img, cv::Size(2 * _params->blur_size + 1, 2 * _params->blur_size + 1), 2, 2);

   cv::Mat hsv_img;
   cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

   cv::Mat element;
   cv::inRange(hsv_img, cv::Scalar(_params->h_lower_th, _params->s_lower_th, _params->v_lower_th),
               cv::Scalar(_params->h_upper_th, _params->s_upper_th, _params->v_upper_th), b_mask);

   int operation = cv::MORPH_OPEN;

   element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size(2 * _params->opening_size + 1, 2 * _params->opening_size + 1),
                                       cv::Point(_params->opening_size, _params->opening_size));

   cv::morphologyEx(b_mask, b_mask_proc, operation, element);
}

void ColorDetector::roiProcessing(cv::Mat& img, cv::Mat& b_mask)
{
   cv::Mat hsv_roi;
   cv::cvtColor(img, hsv_roi, cv::COLOR_BGR2HSV);

   cv::Mat element;
   cv::inRange(hsv_roi, cv::Scalar(_params->h_lower_th, _params->s_lower_th - 50, 0),
               cv::Scalar(_params->h_upper_th, _params->s_upper_th, 255), b_mask);
}

const std::vector<Eigen::Vector3d>& ColorDetector::getDetectionPositions() const { return _detections_positions; }
const std::vector<Eigen::Vector3d>& ColorDetector::getDetectionRelativePositions() const
{
   return _detections_relative_positions;
}

void ColorDetector::setParams(ColorReconfigureParameters* params) { _params = params; }

void ColorDetector::setCameraCalibration(const CameraParameters& params)
{
   *_camera_params = params;
   _georef         = new Georef(params);
}

void ColorDetector::setBaseToCameraTf(const Eigen::Isometry3d& offset)
{
   if (!_georef) return;
   _georef->updateBase2CameraTf(offset);
}

void ColorDetector::updateDronePose(const Eigen::Isometry3d& new_drone_pose)
{
   if (!_georef) return;
   _georef->updateBaseTf(new_drone_pose);
}

// void ColorDetector::drawValidDetections(cv::Mat& img)
// {
//    Eigen::Matrix3f camera_rotation;
//    camera_rotation = Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitX()) *
//                      Eigen::AngleAxisf((0.5 * M_PI), Eigen::Vector3f::UnitZ());

// }

void ColorDetector::geolocaliceValidDetections()
{
   if (!_georef) return;

   _detections_positions.clear();
   _detections_relative_positions.clear();

   for (size_t i = 0; i < _valid_detections.size(); i++)
   {
      Eigen::Vector3d relative_detection;
      Eigen::Vector3d geo_detection =
          _georef->geolocalice(_valid_detections[i].center().x(), _valid_detections[i].center().y(),
                               _valid_detections[i].depth(), relative_detection);

      _valid_detections[i].local_position(geo_detection);

      _detections_positions.push_back(geo_detection);
      _detections_relative_positions.push_back(relative_detection);
   }
}

}  // namespace mbzirc