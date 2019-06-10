#include <hue_object_detection/HueDetection.h>
#include <opencv2/imgproc.hpp>
#include <ros/console.h>
#include <iostream>

// TODO: Make these parameters
#define SATURATION_THR 128
#define LIKEHOOD_THR 96
#define MIN_AREA 2
#define POLY_EPSILON 3

HueDetection::HueDetection() {
	capture_size_ = cvSize(0, 0);
}

void HueDetection::addDetector(const std::string _id, const std::string _histogram_file, cv::Scalar _contour_colour) {

	// Supose in histogram_file we have a colour model (hue) of item
	cv::FileStorage fs(_histogram_file, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::string error = "Unable to open file " + _histogram_file;
		throw std::runtime_error(error);
	}
	cv::Mat histogram;
	fs["histogram"] >> histogram;
	fs.release();

	if (histogram.empty()) {
		std::string error = "Unable to find [histogram] in file " + _histogram_file;
		throw std::runtime_error(error);
	}

	histogram_[_id] = histogram;
	colour_[_id] = _contour_colour;
}

void HueDetection::setFrame(cv::Mat& _frame) {
	if (_frame.empty()) {
		ROS_WARN("HueDetection::setFrame: frame is empty!");
		return;
	}

	// Set capture size
	setImageSize(_frame.size());

	// Transform to HSL space...
	cv::cvtColor(_frame, hls_, CV_BGR2HLS);
	int get_hue[] = {0, 0};
	int get_sat[] = {2, 0};
	cv::mixChannels(&hls_, 3, &hue_, 1, get_hue, 1);  // ...and get hue...
	cv::mixChannels(&hls_, 3, &sat_, 1, get_sat, 1);  // ...and saturation channels.

	// Create a mask based on saturation channel...
	cv::threshold(sat_, satmask_, SATURATION_THR, 255, CV_THRESH_BINARY_INV);
	frame_ = _frame;
}

std::vector<HueItem> HueDetection::detect(const std::string _id, bool _draw) {
	std::vector<HueItem> item_list;
	if (histogram_.count(_id) == 0) {
		ROS_WARN("HueDetection::detect: id [%s] not found!", _id.c_str());
		return item_list;
	}
	if (frame_.empty()) {
		ROS_WARN("HueDetection::detect: frame is empty!");
		return item_list;
	}

	// Create likehood-image based on hue histogram...
    float chan_ranges[] = { 0, 180 };
    const float* ranges[] = { chan_ranges };
    int channels[] = { 0 };
	cv::calcBackProject(&hue_, 1, channels, histogram_[_id], likelihood_, ranges, 255);

	likelihood_.setTo(0, satmask_);  // ...and pass mask.

	// Smooth likelihood image...
	cv::medianBlur(likelihood_, smoothed_, 3);
	cv::threshold(smoothed_, segmented_, LIKEHOOD_THR, 255, CV_THRESH_BINARY);  // .. and segment it.

	// Find contours
  	std::vector<std::vector<cv::Point>> contours;
  	std::vector<cv::Vec4i> hierarchy;
  	findContours(segmented_, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  	/// Draw contours and generate HueItems
	char count_text[33];
  	for (int i = 0; i < contours.size(); i++) {

		cv::Mat polygon;
		cv::approxPolyDP(cv::Mat(contours[i]), polygon, POLY_EPSILON, true);
		double area = fabs(cv::contourArea(polygon));
		if (area < MIN_AREA) { continue; }

		CvMoments moments = cv::moments(polygon);
		cv::Point centroid = cv::Point(cvRound(moments.m10/moments.m00), cvRound(moments.m01/moments.m00));

		if (_draw) {
			cv::Scalar colour = colour_[_id];
			cv::circle(frame_, centroid, 4, colour, -1);
			sprintf(count_text, "%d", i);
			cv::putText(frame_, count_text, centroid, CV_FONT_HERSHEY_PLAIN, 1, colour);
			cv::drawContours(frame_, contours, i, colour, 1);
		}

		HueItem item;
		item.detector_id = _id;
		item.centroid = centroid;
		item.area = area;
		item_list.push_back(item);
     }

	return item_list;
}

std::vector<HueItem> HueDetection::detectAll(bool _draw) {
	std::vector<HueItem> final_list;
	for (std::map<std::string, cv::Mat>::iterator it = histogram_.begin(); it != histogram_.end(); ++it) {
  		std::vector<HueItem> partial_list = detect(it->first, _draw);
		final_list.insert(final_list.end(), partial_list.begin(), partial_list.end());
	}
	return final_list;
}
