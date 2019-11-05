/*!
 * @file color_detector.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 10/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Geometry>

namespace mbzirc
{
class Georef;
class CameraParameters;
class Isometry3d;
class ProcessedImage;
class ColorReconfigureParameters;
class ValidDetection;
class OptFlow;
class OptFlowPointsSet;

class ColorDetector
{
  public:
   ColorDetector(ColorReconfigureParameters* params, const bool print_debug = false);
   virtual ~ColorDetector();
   void processFrame(cv::Mat& img, cv::Mat& dbg1, cv::Mat& dbg2, cv::Mat& dbg3, cv::Mat& dbg4);

   const std::vector<Eigen::Vector3d>& getDetectionPositions() const;
   const std::vector<Eigen::Vector3d>& getDetectionRelativePositions() const;

   void setParams(ColorReconfigureParameters* params);
   void setCameraCalibration(const CameraParameters& params);

   void setBaseToCameraTf(const Eigen::Isometry3d& offset);
   void updateDronePose(const Eigen::Isometry3d& new_drone_pose);
   void geolocaliceValidDetections();

   std::vector<cv::Mat> getThresholdsDebugImages();

  private:
   void imageProcessing(cv::Mat& img, cv::Mat& b_mask, cv::Mat& b_mask_proc);
   void roiProcessing(cv::Mat& img, cv::Mat& b_mask);

   void drawValidDetections(cv::Mat& img);

   // clang-format off
   inline float getCircularity(const float& area, const std::vector<cv::Point>& contour)
   {
      return (float)(4 * M_PI * area / std::pow(cv::arcLength(contour, true), 2));
   }

   inline float getConvexity(const float& area, const int& convex_hull) 
   { 
      return area / std::abs(convex_hull); 
   }

   inline float getDistance(const cv::Point2f& center1, const cv::Point2f& center2)
   {
      return std::sqrt(std::pow(center1.x-center2.x, 2) + std::pow(center1.y-center2.y, 2));
   }
   // clang-format on

   CameraParameters* _camera_params;

   Eigen::Isometry3d _base_to_camera_tf;

   Georef* _georef = {nullptr};

   ColorReconfigureParameters* _params;
   double _max_depth_value;

   cv::Mat _raw_depth_img;
   std::vector<ValidDetection> _valid_detections;
   std::vector<Eigen::Vector3d> _detections_positions;
   std::vector<Eigen::Vector3d> _detections_relative_positions;

   bool _debug;
};

}  // namespace mbzirc