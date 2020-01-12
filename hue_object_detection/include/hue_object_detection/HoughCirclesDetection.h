#ifndef HOUGH_CIRCLES_DETECTION_H
#define HOUGH_CIRCLES_DETECTION_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>

struct CircleItem {
    CircleItem(cv::Vec3f c) {
        x = c[0];
        y = c[1];
        radius = c[2];
    }

    // std::string id;  // TODO: Add?
    double x;
    double y;
    double radius;
};

struct HoughCirclesDetectionConfig {
    double dp = 1;         // Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
    double minDist = 5;    // Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
    double param1 = 100;   // First method-specific parameter. Higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
    double param2 = 100;   // Second method-specific parameter. Accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
    double minRadius = 0;  // Minimum circle radius.
    double maxRadius = 0;  // Maximum circle radius.
};

class HoughCirclesDetection {
public:
    HoughCirclesDetection() { capture_size_ = cvSize(0, 0); }

    void setConfig(const HoughCirclesDetectionConfig& _config) { config_ = _config; }

    void setFrame(cv::Mat& _frame) {
        if (_frame.empty()) {
            ROS_WARN("HoughCirclesDetection::setFrame: frame is empty!");
            return;
        }
        // Set capture size
        setImageSize(_frame.size());
        frame_ = _frame;
    }

    std::vector<CircleItem> detect(bool _draw = false) {
        /// Convert frame to gray
        cvtColor(frame_, blur_, CV_BGR2GRAY);

        /// Reduce the noise so we avoid false circle detection
        GaussianBlur(blur_, blur_, cv::Size(9, 9), 2, 2);

        /// Apply the Hough Transform to find the circles
        std::vector<cv::Vec3f> circles;
        HoughCircles(blur_, circles, CV_HOUGH_GRADIENT, config_.dp, config_.minDist, \
            config_.param1, config_.param2, config_.minRadius, config_.maxRadius);

        std::vector<CircleItem> item_list;
        for (size_t i = 0; i < circles.size(); i++) {
            CircleItem c(circles[i]);
            item_list.push_back(c);
            if (_draw && i == 0) {  // TODO: Only first?
                cv::Point center(cvRound(c.x), cvRound(c.y));
                int radius = cvRound(c.radius);
                circle(frame_, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);  // center
                circle(frame_, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);  // outline
            }
        }
        return item_list;
    }

private:

    void setImageSize(const cv::Size& _size) {
        if((capture_size_.width != _size.width) || (capture_size_.height != _size.height)) {
            blur_ = cv::Mat(_size, CV_8UC1);
            capture_size_ = _size;
        }
    }

    cv::Mat frame_, blur_;
    cv::Size capture_size_;

    HoughCirclesDetectionConfig config_;
};

#endif  // HOUGH_CIRCLES_DETECTION_H
