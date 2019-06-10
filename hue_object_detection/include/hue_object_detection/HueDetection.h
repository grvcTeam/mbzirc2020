#ifndef HUE_DETECTION_H
#define HUE_DETECTION_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>

struct HueItem {
	std::string detector_id;
	cv::Point centroid;
	double area;
};

class HueDetection {
public:
	HueDetection();
	void addDetector(const std::string _id, const std::string _histogram_file, cv::Scalar _contour_colour);
	void setFrame(cv::Mat& _frame);
	std::vector<HueItem> detect(const std::string _id, bool _draw = false);
	std::vector<HueItem> detectAll(bool _draw = false);

private:
	std::map<std::string, cv::Mat> histogram_;
	std::map<std::string, cv::Scalar> colour_;

	void setImageSize(const cv::Size& _size);
	cv::Mat frame_, hls_, hue_, sat_, satmask_;
	cv::Mat likelihood_, smoothed_, segmented_;
	cv::Size capture_size_;
};

inline void HueDetection::setImageSize(const cv::Size& _size) {
	if((capture_size_.width != _size.width) || (capture_size_.height != _size.height)) {
		hls_ = cv::Mat(_size, CV_8UC3);
		hue_ = cv::Mat(_size, CV_8UC1);
		sat_ = cv::Mat(_size, CV_8UC1);
		satmask_ = cv::Mat(_size, CV_8UC1);
		likelihood_ = cv::Mat(_size, CV_8UC1);
		segmented_ = cv::Mat(_size, CV_8UC1);
		smoothed_ = cv::Mat(_size, CV_8UC1);
		capture_size_ = _size;
	}
}

#endif  // HUE_DETECTION_H
