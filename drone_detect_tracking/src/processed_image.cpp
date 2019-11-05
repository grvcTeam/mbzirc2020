/*!
 * @file processed_image.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 15/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <opencv2/imgproc/imgproc.hpp>

#include <drone_detector_tracking/candidate.h>
#include <drone_detector_tracking/processed_image.h>

namespace mbzirc
{
ProcessedImage::ProcessedImage(const cv::Mat& mask, const int& id)
{
   _bw_mask = mask.clone();
   cv::cvtColor(_bw_mask * 255, _color_mask, CV_GRAY2RGB);

   _id = id;
}

ProcessedImage::~ProcessedImage() {}

void ProcessedImage::addDetection(const Candidate& detection)
{
   _candidates.push_back(detection);
   _contours.push_back(detection.contour());
}

const size_t& ProcessedImage::candidatesSize() const { return _candidates.size(); }

cv::Mat ProcessedImage::getColorMask() const { return _color_mask; }
cv::Mat ProcessedImage::getMask() const { return _bw_mask; }

cv::Mat ProcessedImage::getCandidatesContoursMask() const
{
   cv::Mat candidates_mask(cv::Size(_bw_mask.cols, _bw_mask.rows), CV_8UC1, cv::Scalar::all(0));
   cv::drawContours(candidates_mask, _contours, -1, cv::Scalar(1), CV_FILLED);

   return candidates_mask;
}

const std::vector<Candidate>& ProcessedImage::getCandidates() const { return _candidates; }

const std::vector<std::vector<cv::Point>>& ProcessedImage::getCandidatesContours() const { return _contours; }

const int& ProcessedImage::id() const { return _id; }

void ProcessedImage::groupCandidates(int max_dist)
{
   const int max_dist2 = max_dist * max_dist;

   std::vector<int> groups;
   int groups_size =
       cv::partition(_candidates, groups, [max_dist2](const Candidate& candidate1, const Candidate& candidate2) {
          return (pow(candidate2.contourCenter().x - candidate1.contourCenter().x, 2) +
                  pow(candidate2.contourCenter().y - candidate1.contourCenter().y, 2)) < max_dist2;
       });

   std::vector<cv::Point2f> center_sum(groups_size, cv::Point2f(0.0f, 0.0f));
   std::vector<float> circularity_sum(groups_size, 0.0f);
   std::vector<float> convexity_sum(groups_size, 0.0f);
   std::vector<int> count(groups_size, 0);

   for (size_t i = 0; i < _candidates.size(); i++)
   {
      const int index = groups[i];

      center_sum[index].x += _candidates[i].contourCenter().x;
      center_sum[index].y += _candidates[i].contourCenter().y;
      circularity_sum[index] += _candidates[i].circularity();
      convexity_sum[index] += _candidates[i].convexity();
      count[index]++;
   }

   std::vector<Candidate> grouped_candidates;
   for (size_t i = 0; i < groups_size; i++)
   {
      Candidate candidate;
      candidate.contourCenter(cv::Point2f(center_sum[i].x / count[i], center_sum[i].y / count[i]));
      candidate.circularity(circularity_sum[i] / count[i]);
      candidate.convexity(convexity_sum[i] / count[i]);
      grouped_candidates.push_back(candidate);
   }

   _candidates = grouped_candidates;
}

}  // namespace mbzirc