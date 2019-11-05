/*!
 * @file candidate.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 16/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <opencv2/imgproc/imgproc.hpp>

#include <drone_detector_tracking/candidate.h>

namespace mbzirc
{
Candidate::Candidate() {}
Candidate::Candidate(const std::vector<cv::Point>& contour, const float& circularity, const float& convexity)
{
   this->contour(contour);
   this->circularity(circularity);
   this->convexity(convexity);
}

Candidate::~Candidate() {}

const std::vector<cv::Point>& Candidate::contour() const { return _contour; }
void Candidate::contour(const std::vector<cv::Point>& contour)
{
   _contour = contour;

   const cv::Moments mu = cv::moments(contour);

   _contour_center =
       cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)), static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));
}

const cv::Point2f& Candidate::contourCenter() const { return _contour_center; }
void Candidate::contourCenter(const cv::Point2f& contour_center) { _contour_center = contour_center; }

const float& Candidate::circularity() const { return _circularity; }
void Candidate::circularity(const float& circularity) { _circularity = circularity; }

const float& Candidate::convexity() const { return _convexity; }
void Candidate::convexity(const float& convexity) { _convexity = convexity; }

}  // namespace mbzirc