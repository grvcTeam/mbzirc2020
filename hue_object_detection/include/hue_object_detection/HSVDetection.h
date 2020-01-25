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
#define KERNEL_MAX_TYPE_COUNT 2
#define KERNEL_MAX_SIZE 21

struct Kernel {
  int type = 0;
  int size = 0;

  cv::Mat getAsElement() {
    int out_type;
    switch (type) {
      case 0: out_type = cv::MORPH_RECT; break;
      case 1: out_type = cv::MORPH_CROSS; break;
      case 2: out_type = cv::MORPH_ELLIPSE; break;
      default: throw std::runtime_error("Unexpected kernel type!");
    }
    return getStructuringElement(out_type, cv::Size(2*size + 1, 2*size + 1), cv::Point(size, size));
  }
};

struct HSVRange {
    int min_HSV[3] = {0, 0, 0};
    int max_HSV[3] = {MAX_VALUE_H, MAX_VALUE_S, MAX_VALUE_V};

    cv::Scalar getMin() { return cv::Scalar(min_HSV[0], min_HSV[1], min_HSV[2]); }
    cv::Scalar getMax() { return cv::Scalar(max_HSV[0], max_HSV[1], max_HSV[2]); }
};

struct HSVItem {
    std::string detector_id;
    cv::RotatedRect rectangle;
    // cv::Point centroid;
    // double area;
    // double perimeter;
    // double orientation;  // wrt horizontal axis, cw-positive (wrt x-axis at cv_frame)
    // contour;
    bool cropped = false;
};

struct HSVDetectionConfig {
    double min_area = 2.0;
    double poly_epsilon = 3.0;
    Kernel kernel;
};

class HSVDetection {
public:
    HSVDetection();
    void setConfig(const HSVDetectionConfig& _config);
    void addDetector(const std::string _id, const HSVRange _range, cv::Scalar _contour_colour);
    void setFrame(cv::Mat& _frame);
    std::vector<HSVItem> detect(const std::string _id, bool _draw = false);
    std::vector<HSVItem> detectAll(bool _draw = false);

    std::vector<HSVItem> track(const std::string _id, bool _draw = false);
    cv::Mat getDetection() { return in_range_; }  // Debug!

private:
    std::vector<HSVItem> detectPipeline(const std::string _id, const cv::Mat& _src, cv::Mat& _visual, bool _draw = false);
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

std::vector<HSVItem> HSVDetection::detectPipeline(const std::string _id, const cv::Mat& _src, cv::Mat& _visual, bool _draw) {
     std::vector<HSVItem> item_list;
    if (range_.count(_id) == 0) {
        ROS_WARN("HSVDetection::detectPipeline: id [%s] not found!", _id.c_str());
        return item_list;
    }
    if (_src.empty()) {
        ROS_WARN("HSVDetection::detectPipeline: source is empty!");
        return item_list;
    }

    // Detect the object based on HSV Range Values
    cv::Mat in_range = cv::Mat(capture_size_, CV_8UC1);
    inRange(_src, range_[_id].getMin(), range_[_id].getMax(), in_range);

    // Smooth image...
    morphologyEx(in_range, in_range, cv::MORPH_OPEN, config_.kernel.getAsElement());

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(in_range, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    /// Draw contours and generate HSVItems
    char count_text[33];
    int valid_index = 0;
    for (int i = 0; i < contours.size(); i++) {

        cv::Mat polygon;
        cv::approxPolyDP(cv::Mat(contours[i]), polygon, config_.poly_epsilon, true);
        double area = fabs(cv::contourArea(polygon));
        if (area < config_.min_area) { continue; }

        cv::RotatedRect rect = minAreaRect(polygon);  // TODO: contours[i]? polygon?
        // CvMoments moments = cv::moments(polygon);
        // cv::Point centroid = cv::Point(cvRound(moments.m10/moments.m00), cvRound(moments.m01/moments.m00));
        // double mu20_prime = moments.mu20 / moments.m00;  // mu00 = m00
        // double mu02_prime = moments.mu02 / moments.m00;  // mu00 = m00
        // double mu11_prime = moments.mu11 / moments.m00;  // mu00 = m00
        // double theta = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);

        if (_draw) {
             cv::Scalar colour = colour_[_id];
            // cv::circle(_visual, centroid, 4, colour, -1);
            // sprintf(count_text, "%d", valid_index++);
            // cv::putText(_visual, count_text, centroid, CV_FONT_HERSHEY_PLAIN, 1, colour);
            cv::drawContours(_visual, polygon, -1, colour, 3);
            // Draw rect:
            cv::circle(_visual, rect.center, 4, colour, -1);
            sprintf(count_text, "%d", valid_index++);
            cv::putText(_visual, count_text, rect.center, CV_FONT_HERSHEY_PLAIN, 1, colour);
            cv::Point2f vertices[4];  // TODO: draw function
            rect.points(vertices);
            for (int j = 0; j < 4; j++) {
                line(_visual, vertices[j], vertices[(j+1)%4], colour, 2);
            }
        }

        HSVItem item;
        item.detector_id = _id;
        item.rectangle = rect;
        // item.centroid = centroid;
        // item.area = area;
        // item.perimeter = cv::arcLength(polygon, true);
        // item.orientation = theta;
        item_list.push_back(item);
     }

    return item_list;   
}

std::vector<HSVItem> HSVDetection::detect(const std::string _id, bool _draw) {
    return detectPipeline(_id, hsv_, frame_, _draw);
}

std::vector<HSVItem> HSVDetection::detectAll(bool _draw) {
    std::vector<HSVItem> final_list;
    for (auto it = range_.begin(); it != range_.end(); ++it) {
          std::vector<HSVItem> partial_list = detect(it->first, _draw);
        final_list.insert(final_list.end(), partial_list.begin(), partial_list.end());
    }
    return final_list;
}

std::vector<HSVItem> HSVDetection::track(const std::string _id, bool _draw) {

    std::vector<HSVItem> item_list;
    std::vector<HSVItem> detected = detect(_id, _draw);  // TODO: draw?
    if (detected.size() == 0) { return item_list; }

    cv::Size hsv_size = hsv_.size();
    // cv::Point2f target_point = cv::Point2f(0.5 * hsv_size.width, 0.5 * hsv_size.height);

    HSVItem closest;
    float min_sq_distance = (hsv_size.width + hsv_size.width) * (hsv_size.width + hsv_size.width);
    for (auto item: detected) {
        // ROS_ERROR("angle = %f, center = (%f, %f), size = (%f, %f)", item.rectangle.angle, item.rectangle.center.x, item.rectangle.center.y, item.rectangle.size.width, item.rectangle.size.height);
        // ROS_ERROR("hsv_size = (%d, %d)", hsv_size.width, hsv_size.height);
        float dx = fabs(hsv_size.width/2  - item.rectangle.center.x);
        float dy = fabs(hsv_size.height/2 - item.rectangle.center.y);
        float sq_distance = dx*dx + dy*dy;
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            closest = item;
        }
    }
    // ROS_ERROR("angle = %f, center = (%f, %f), size = (%f, %f)", closest.rectangle.angle, closest.rectangle.center.x, closest.rectangle.center.y, closest.rectangle.size.width, closest.rectangle.size.height);

    cv::Rect roi = closest.rectangle.boundingRect() & cv::Rect(0, 0, hsv_size.width, hsv_size.height);
    closest.cropped = (closest.rectangle.boundingRect() != roi);
    if ((roi.width <= 0) || (roi.height <= 0)) { return item_list; }
    if (_draw) {
        rectangle(frame_, closest.rectangle.boundingRect(), colour_[_id]);
    }

    // ROS_ERROR("(x, y) = (%d, %d), (w, h) = (%d, %d)", roi.x, roi.y, roi.width, roi.height);

    // range_["white"] is mandatory here!
    if (range_.count("white") == 0) {
        ROS_WARN("HSVDetection::track: id [white] not found!");
        return item_list;
    }
    cv::Mat hsv_roi = hsv_(roi);
    cv::Mat in_range_roi;

    std::vector<HSVItem> white_list = detectPipeline("white", hsv_roi, frame_, _draw);

    return item_list;
}

#endif  // HSV_DETECTION_H
