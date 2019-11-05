/*!
 * @file processed_image.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 16/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <opencv2/core/core.hpp>

namespace mbzirc
{
class Candidate;

class ProcessedImage
{
  public:
   ProcessedImage(const cv::Mat& mask, const int& id);
   virtual ~ProcessedImage();

   void addDetection(const Candidate& detection);

   const size_t& candidatesSize() const;

   cv::Mat getColorMask() const;
   cv::Mat getMask() const;

   cv::Mat getCandidatesContoursMask() const;

   const std::vector<Candidate>& getCandidates() const;
   const std::vector<std::vector<cv::Point>>& getCandidatesContours() const;

   const int& id() const;

   void groupCandidates(int max_dist = 100);

  private:
   cv::Mat _bw_mask;
   cv::Mat _color_mask;

   std::vector<Candidate> _candidates;
   std::vector<std::vector<cv::Point>> _contours;

   int _id;
};

}  // namespace mbzirc