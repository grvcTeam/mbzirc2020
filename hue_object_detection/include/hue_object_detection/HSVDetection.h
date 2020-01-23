#ifndef HSV_DETECTION_H
#define HSV_DETECTION_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>

#include <opencv2/imgproc.hpp>
#include <ros/console.h>
#include <iostream>

#define MAX_VALUE_H 180  // 180 = 360/2
#define MAX_VALUE_S 255
#define MAX_VALUE_V 255

struct HSVRange {
	int min_HSV[3] = {0, 0, 0};
	int max_HSV[3] = {MAX_VALUE_H, MAX_VALUE_S, MAX_VALUE_V};

	cv::Scalar getMin() { return cv::Scalar(min_HSV[0], min_HSV[1], min_HSV[2]); }
	cv::Scalar getMax() { return cv::Scalar(max_HSV[0], max_HSV[1], max_HSV[2]); }
};

struct HSVItem {
	std::string detector_id;
	cv::Point centroid;
	double area;
	double perimeter;
	double orientation;  // wrt horizontal axis, cw-positive (wrt x-axis at cv_frame)
	// contour;
	// bool cliped = false;
};

struct HSVDetectionConfig {
	double saturation_threshold = 128.0;
	double likelihood_threshold = 96.0;
	double min_area = 2.0;
	double poly_epsilon = 3.0;
};

class HSVDetection {
public:
	HSVDetection();
	void setConfig(const HSVDetectionConfig& _config);
	void addDetector(const std::string _id, const HSVRange _range, cv::Scalar _contour_colour);
	void setFrame(cv::Mat& _frame);
	std::vector<HSVItem> detect(const std::string _id, bool _draw = false);
	std::vector<HSVItem> detectAll(bool _draw = false);

	cv::Mat getDetection() { return in_range_; }

private:
	std::map<std::string, HSVRange> range_;
	std::map<std::string, cv::Scalar> colour_;

	void setImageSize(const cv::Size& _size);
	cv::Mat frame_, hsv_, in_range_;
	cv::Size capture_size_;

	HSVDetectionConfig config_;
};

inline void HSVDetection::setImageSize(const cv::Size& _size) {
	if((capture_size_.width != _size.width) || (capture_size_.height != _size.height)) {
		hsv_ = cv::Mat(_size, CV_8UC3);
		in_range_ = cv::Mat(_size, CV_8UC1);
		capture_size_ = _size;
	}
}

HSVDetection::HSVDetection() {
	capture_size_ = cvSize(0, 0);
}

void HSVDetection::setConfig(const HSVDetectionConfig& _config) { config_ = _config; }

void HSVDetection::addDetector(const std::string _id, const HSVRange _range, cv::Scalar _contour_colour) {
	// TODO: check HSVRange consistency
	range_[_id] = _range;
	colour_[_id] = _contour_colour;
}

void HSVDetection::setFrame(cv::Mat& _frame) {
	if (_frame.empty()) {
		ROS_WARN("HSVDetection::setFrame: frame is empty!");
		return;
	}

	// Set capture size
	setImageSize(_frame.size());

	// Transform to HSV space...
    cvtColor(_frame, hsv_, cv::COLOR_BGR2HSV);

	frame_ = _frame;
}

std::vector<HSVItem> HSVDetection::detect(const std::string _id, bool _draw) {
	std::vector<HSVItem> item_list;
	if (range_.count(_id) == 0) {
		ROS_WARN("HSVDetection::detect: id [%s] not found!", _id.c_str());
		return item_list;
	}
	if (frame_.empty()) {
		ROS_WARN("HSVDetection::detect: frame is empty!");
		return item_list;
	}

	// Detect the object based on HSV Range Values
	inRange(hsv_, range_[_id].getMin(), range_[_id].getMax(), in_range_);

	// Smooth likelihood image...
	// cv::medianBlur(likelihood_, smoothed_, 3);
	// cv::threshold(smoothed_, segmented_, config_.likelihood_threshold, 255, CV_THRESH_BINARY);  // .. and segment it.
    // mask = cv2.erode(mask, None, iterations=2)
    // mask = cv2.dilate(mask, None, iterations=2)

    // ## BEGIN - draw rotated rectangle
    // rect = cv2.minAreaRect(c)
    // box = cv2.boxPoints(rect)
    // box = np.int0(box)
    // cv2.drawContours(image,[box],0,(0,191,255),2)
    // ## END - draw rotated rectangle

	// Find contours
  	std::vector<std::vector<cv::Point>> contours;
  	std::vector<cv::Vec4i> hierarchy;
  	findContours(in_range_, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  	/// Draw contours and generate HSVItems
	char count_text[33];
  	for (int i = 0; i < contours.size(); i++) {

		cv::RotatedRect rect = minAreaRect(contours[i]);
		// cv::Mat polygon;
		// cv::approxPolyDP(cv::Mat(contours[i]), polygon, config_.poly_epsilon, true);
		// double area = fabs(cv::contourArea(polygon));
		// if (area < config_.min_area) { continue; }

		// CvMoments moments = cv::moments(polygon);
		// cv::Point centroid = cv::Point(cvRound(moments.m10/moments.m00), cvRound(moments.m01/moments.m00));
		// double mu20_prime = moments.mu20 / moments.m00;  // mu00 = m00
		// double mu02_prime = moments.mu02 / moments.m00;  // mu00 = m00
		// double mu11_prime = moments.mu11 / moments.m00;  // mu00 = m00
		// double theta = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);

		// if (_draw) {
		// 	cv::Scalar colour = colour_[_id];
		// 	cv::circle(frame_, centroid, 4, colour, -1);
		// 	sprintf(count_text, "%d", i);
		// 	cv::putText(frame_, count_text, centroid, CV_FONT_HERSHEY_PLAIN, 1, colour);
		// 	cv::drawContours(frame_, contours, i, colour, 1);
		// }

		// HSVItem item;
		// item.detector_id = _id;
		// item.centroid = centroid;
		// item.area = area;
		// item.perimeter = cv::arcLength(polygon, true);
		// item.orientation = theta;
		// item_list.push_back(item);
     }

	return item_list;
}

std::vector<HSVItem> HSVDetection::detectAll(bool _draw) {
	// std::vector<HSVItem> final_list;
	// for (std::map<std::string, cv::Mat>::iterator it = range_.begin(); it != range_.end(); ++it) {
  	// 	std::vector<HSVItem> partial_list = detect(it->first, _draw);
	// 	final_list.insert(final_list.end(), partial_list.begin(), partial_list.end());
	// }
	// return final_list;
}

#endif  // HSV_DETECTION_H
