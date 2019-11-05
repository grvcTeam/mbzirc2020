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

#include <opencv2/core/core.hpp>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>
#include <drone_detector_tracking/candidate.h>
#include <drone_detector_tracking/drone_detector.h>
#include <drone_detector_tracking/georef.h>
#include <drone_detector_tracking/opt_flow.h>
#include <drone_detector_tracking/opt_flow_points_set.h>
#include <drone_detector_tracking/processed_image.h>
#include <drone_detector_tracking/reconfigure_parameters.h>
#include <drone_detector_tracking/valid_detection.h>

namespace mbzirc
{
DroneDetector::DroneDetector(ReconfigureParameters* params, const bool print_debug)
{
   _debug = print_debug;
   setParams(params);
}

DroneDetector::~DroneDetector() {}

void DroneDetector::processFrame(cv::Mat& depth_img, cv::Mat& rgb_img)
{
   _raw_depth_img = depth_img.clone();

   preprocessing(depth_img);
   depthThresholding(depth_img);
   postprocessing();
   geolocaliceValidDetections();
   drawValidDetections(rgb_img);
}

const std::vector<Eigen::Vector3d>& DroneDetector::getDetectionPositions() const { return _detections_positions; }
const std::vector<Eigen::Vector3d>& DroneDetector::getDetectionRelativePositions() const
{
   return _detections_relative_positions;
}

void DroneDetector::setParams(ReconfigureParameters* params) { _params = params; }

void DroneDetector::setCameraCalibration(const CameraParameters& params) { _georef = new Georef(params); }

void DroneDetector::setBaseToCameraTf(const Eigen::Isometry3d& offset)
{
   if (!_georef) return;
   _georef->updateBase2CameraTf(offset);
}

void DroneDetector::updateDronePose(const Eigen::Isometry3d& camera_pose)
{
   if (!_georef) return;
   _georef->updateBaseTf(camera_pose);

   _params->max_depth = camera_pose.translation()[2];
}

void DroneDetector::preprocessing(cv::Mat& img)
{
   cv::minMaxLoc(img, nullptr, &_max_depth_value);

   cv::Mat unkwown_pixels = (img == 0.);
   unkwown_pixels.convertTo(unkwown_pixels, CV_32FC1);

   img = img + _max_depth_value * unkwown_pixels;

   cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * _params->erosion_size + 1, 2 * _params->erosion_size + 1),
                                               cv::Point(_params->erosion_size, _params->erosion_size));

   cv::erode(img, img, element);

   img = img * _params->unit_depth;
}

void DroneDetector::depthThresholding(cv::Mat& img)
{
   _processed_images.clear();

   if (_max_depth_value > _params->max_depth) _max_depth_value = _params->max_depth;

   const size_t iterations = (int)_max_depth_value / _params->step_depth;

   float depth_threshold = _params->step_depth;
   for (size_t i = 0; i < iterations; i++)
   {
      cv::Mat mask;
      cv::threshold(img, mask, depth_threshold, 1, cv::THRESH_BINARY_INV);

      mask.convertTo(mask, CV_8UC1);

      ProcessedImage processed_image(mask, i);

      depth_threshold += _params->step_depth;

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      for (size_t j = 0; j < contours.size(); j++)
      {
         const float area = cv::contourArea(contours[j]);  // px
         if (area < _params->min_area || area > _params->max_area) continue;

         const float circularity = getCircularity(area, contours[j]);
         if (circularity < _params->min_circularity || circularity > _params->max_circularity) continue;

         std::vector<cv::Point> hull;
         convexHull(cv::Mat(contours[j]), hull, false);

         const float convexity = getConvexity(area, std::abs(cv::contourArea(hull)));
         if (convexity < _params->min_convexity || convexity > _params->max_convexity) continue;

         processed_image.addDetection(Candidate(contours[j], circularity, convexity));
      }

      processed_image.groupCandidates(_params->max_dist_local_detections);
      _processed_images.push_back(processed_image);
   }
}

void DroneDetector::postprocessing()
{
   _valid_detections.clear();

   const size_t processed_imgs_size = _processed_images.size();
   const size_t min_group_size      = _params->min_group_size;
   const size_t max_dist            = _params->max_dist_detections;

   int max_dist2 = max_dist * max_dist;

   if (processed_imgs_size < min_group_size) return;

   for (size_t i = 0; i < processed_imgs_size - min_group_size + 1; i++)  // window sliding iterations
   {
      // Grouping candidates in the same vector with their ID
      std::vector<std::pair<int, Candidate>> window_candidates;
      for (size_t j = 0; j < min_group_size; j++)
      {
         const int k  = i + j;
         const int id = _processed_images[k].id();

         const std::vector<Candidate> candidates = _processed_images[k].getCandidates();
         for (auto candidate : candidates)
         {
            window_candidates.push_back(std::make_pair(id, candidate));
         }
      }

      // Finding groups with the max_dist criterion
      std::vector<int> groups;
      int groups_size = cv::partition(
          window_candidates, groups,
          [max_dist2](const std::pair<int, Candidate>& candidate1, const std::pair<int, Candidate>& candidate2) {
             return (pow(candidate2.second.contourCenter().x - candidate1.second.contourCenter().x, 2) +
                     pow(candidate2.second.contourCenter().y - candidate1.second.contourCenter().y, 2)) < max_dist2;
          });

      // Saving results all together
      std::vector<std::set<int>> grouped_ids(groups_size);
      std::vector<cv::Point2f> grouped_center(groups_size);

      std::vector<cv::Point2f> center_sum(groups_size, cv::Point2f(0.0f, 0.0f));
      std::vector<int> count(groups_size, 0);
      for (size_t j = 0; j < window_candidates.size(); j++)
      {
         const int index = groups[j];

         grouped_ids[index].insert(window_candidates[j].first);

         center_sum[index].x += window_candidates[j].second.contourCenter().x;
         center_sum[index].y += window_candidates[j].second.contourCenter().y;
         count[index]++;
      }

      for (size_t j = 0; j < grouped_ids.size(); j++)
      {
         if (grouped_ids[j].size() != min_group_size) continue;

         float depth = cv::Mat(_raw_depth_img * _params->unit_depth)
                           .at<float>(cv::Point(center_sum[j].x / count[j], center_sum[j].y / count[j]));

         ValidDetection valid_detection(center_sum[j].x / count[j], center_sum[j].y / count[j], depth,
                                        _params->sigma_xy, _params->sigma_z);

         _valid_detections.push_back(valid_detection);
      }
   }

   // Finding groups in the final vector
   max_dist2 = pow(_params->max_dist_local_detections, 2);

   std::vector<int> groups;
   int groups_size = cv::partition(_valid_detections, groups,
                                   [max_dist2](const ValidDetection& detection1, const ValidDetection& detection2) {
                                      return (pow(detection2.center().x() - detection1.center().x(), 2) +
                                              pow(detection2.center().y() - detection1.center().y(), 2)) < max_dist2;
                                   });

   std::vector<cv::Point2f> center_sum(groups_size, cv::Point2f(0.0f, 0.0f));
   std::vector<int> count(groups_size, 0);

   for (size_t i = 0; i < _valid_detections.size(); i++)
   {
      const int index = groups[i];

      center_sum[index].x += _valid_detections[i].center().x();
      center_sum[index].y += _valid_detections[i].center().y();
      count[index]++;
   }

   std::vector<ValidDetection> grouped_detections;
   for (size_t i = 0; i < groups_size; i++)
   {
      float depth = cv::Mat(_raw_depth_img * _params->unit_depth)
                        .at<float>(cv::Point(center_sum[i].x / count[i], center_sum[i].y / count[i]));

      ValidDetection valid_detection(center_sum[i].x / count[i], center_sum[i].y / count[i], depth, _params->sigma_xy,
                                     _params->sigma_z);

      if (valid_detection.depth() < 10.0)
      {
         grouped_detections.push_back(valid_detection);
      }
   }

   _valid_detections = grouped_detections;
}

void DroneDetector::geolocaliceValidDetections()
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

void DroneDetector::drawValidDetections(cv::Mat& img)
{
   Eigen::Matrix3f camera_rotation;
   camera_rotation = Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitX()) *
                     Eigen::AngleAxisf((0.5 * M_PI), Eigen::Vector3f::UnitZ());

   int radius = 20;
   for (auto detection : _valid_detections)
   {
      cv::Scalar color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
      cv::Point2f center(detection.center()[0], detection.center()[1]);
      cv::circle(img, center, radius, color, 5);
      Eigen::Vector3d geoposition = detection.local_position();
      std::string geoposition_str = "depth: " + std::to_string(detection.depth());
      cv::putText(img, geoposition_str, center - cv::Point2f(radius * 2, -radius * 2), cv::FONT_HERSHEY_PLAIN, 2, color,
                  2);
   }
}

std::vector<cv::Mat> DroneDetector::getThresholdsDebugImages()
{
   if (_processed_images.size() == 0) return std::vector<cv::Mat>();

   std::vector<cv::Mat> result_debug_images;

   const size_t rows   = 4;
   const size_t colums = 4;

   const unsigned int groups = (_processed_images.size() / (rows * colums)) + 1;
   for (size_t k = 0; k < groups; k++)
   {
      cv::Mat canvas_img = _processed_images[0].getColorMask();
      cv::Mat debug_image(cv::Size((canvas_img.cols + 1) * 4, (canvas_img.rows + 1) * 4), CV_8UC3, cv::Scalar::all(0));

      size_t iters = 0;
      for (size_t i = 0; i < rows; i++)
      {
         for (size_t j = 0; j < colums; j++)
         {
            size_t index = (k * rows * colums) + iters;
            iters++;

            if (index >= _processed_images.size()) break;

            const cv::Point2f offset((canvas_img.cols) * j, (canvas_img.rows) * i);

            cv::Mat roi = debug_image(
                cv::Rect((canvas_img.cols + 1) * j, (canvas_img.rows + 1) * i, canvas_img.cols, canvas_img.rows));

            canvas_img = _processed_images[index].getColorMask();

            for (auto candidate : _processed_images[index].getCandidates())
            {
               cv::circle(canvas_img, candidate.contourCenter(), 5,
                          cv::Scalar(rand() & 255, rand() & 255, rand() & 255), 5);
            }

            canvas_img.copyTo(roi);
         }
      }

      for (size_t j = 1; j < colums; j++)
      {
         const int x_px = (canvas_img.cols + 1) * j;

         cv::line(debug_image, cv::Point(x_px, 0), cv::Point(x_px, (canvas_img.rows + 1) * 4), cv::Scalar(255, 255, 51),
                  3);
      }

      for (size_t i = 1; i < rows; i++)
      {
         const int y_px = (canvas_img.rows + 1) * i;

         cv::line(debug_image, cv::Point(0, y_px), cv::Point((canvas_img.cols + 1) * 4, y_px), cv::Scalar(255, 255, 51),
                  3);
      }

      result_debug_images.push_back(debug_image);
   }

   return result_debug_images;
}
}  // namespace mbzirc