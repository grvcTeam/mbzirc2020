/*!
 * @file candidate.h
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
class Candidate
{
  public:
   Candidate();
   Candidate(const std::vector<cv::Point>& contour, const float& circularity, const float& convexity);

   virtual ~Candidate();

   const std::vector<cv::Point>& contour() const;
   void contour(const std::vector<cv::Point>& contour);

   const cv::Point2f& contourCenter() const;
   void contourCenter(const cv::Point2f& contour_center);

   const float& circularity() const;
   void circularity(const float& circularity);

   const float& convexity() const;
   void convexity(const float& convexity);

  private:
   std::vector<cv::Point> _contour;
   cv::Point2f _contour_center;

   float _circularity;
   float _convexity;
};

}  // namespace mbzirc