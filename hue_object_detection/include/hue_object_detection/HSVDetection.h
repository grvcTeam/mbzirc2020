#ifndef HSV_DETECTION_H
#define HSV_DETECTION_H

#include <yaml-cpp/yaml.h>
#include <ros/console.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <map>

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

    void print() {
        printf("[%d, %d][%d, %d][%d, %d]\n", min_HSV[0], max_HSV[0], min_HSV[1], max_HSV[1], min_HSV[2], max_HSV[2]);
    }
};

void operator>>(const YAML::Node& _in, HSVRange& _range) {
    // TODO: Check that expected fields do exist
    _range.min_HSV[0] = _in["H_range"][0].as<int>();
    _range.min_HSV[1] = _in["S_range"][0].as<int>();
    _range.min_HSV[2] = _in["V_range"][0].as<int>();
    _range.max_HSV[0] = _in["H_range"][1].as<int>();
    _range.max_HSV[1] = _in["S_range"][1].as<int>();
    _range.max_HSV[2] = _in["V_range"][1].as<int>();
}

struct HSVItem {
    std::string detector_id;
    cv::RotatedRect rectangle;
    bool cropped = false;
};

struct HSVTrackingPair {
    HSVItem colour_item;
    HSVItem white_item;
    cv::Point2f white_edge_center;
    bool has_colour = false;
    bool has_white = false;

    void print() {
        printf("[%s]: (%f, %f), [%f, %f], %fº%s\n", colour_item.detector_id.c_str(), colour_item.rectangle.center.x, colour_item.rectangle.center.y, colour_item.rectangle.size.width, colour_item.rectangle.size.height, colour_item.rectangle.angle, colour_item.cropped? ", cropped!": "");
        printf("[%s]: (%f, %f), [%f, %f], %fº%s\n", white_item.detector_id.c_str(), white_item.rectangle.center.x, white_item.rectangle.center.y, white_item.rectangle.size.width, white_item.rectangle.size.height, white_item.rectangle.angle, white_item.cropped? ", cropped!": "");
    }
};

// TODO: Give consistency to rectangle sizes and orientation
cv::RotatedRect sanitizeRotatedRect(const cv::RotatedRect& _rect) {
    cv::RotatedRect out = _rect;
    if (_rect.size.height < _rect.size.width) {
        out.size.height = _rect.size.width;
        out.size.width  = _rect.size.height;
        out.angle = _rect.angle + 90;
    }

    return out;
}

struct HSVDetectionConfig {
    double min_area = 2.0;
    double poly_epsilon = 3.0;
    Kernel kernel;
};

void operator>>(const YAML::Node& _in, HSVDetectionConfig& _config) {
    // TODO: Check that expected fields do exist
    _config.min_area     = _in["min_area"].as<double>();
    _config.poly_epsilon = _in["poly_epsilon"].as<double>();
    _config.kernel.type  = _in["kernel_type"].as<int>();
    _config.kernel.size  = _in["kernel_size"].as<int>();
}

class HSVDetection {
public:
    HSVDetection();
    void setConfig(const HSVDetectionConfig& _config);
    void addDetector(const std::string _id, const HSVRange _range, cv::Scalar _contour_colour);
    void setFrame(cv::Mat& _frame);
    std::vector<HSVItem> detect(const std::string _id, bool _draw = false);
    std::vector<HSVItem> detectAll(bool _draw = false);

    std::vector<HSVItem> track(const std::string _id, bool _draw = false);
    HSVTrackingPair trackBrick(const std::string _id, bool _draw = false);
    void stopTracking() { tracking_ = false; }

private:
    std::vector<HSVItem> detectPipeline(const std::string _id, const cv::Mat& _src, bool _draw = false);
    void drawRotatedRect(const cv::RotatedRect& _r, int _index, cv::Scalar _colour);
    bool is_cropped(const cv::RotatedRect& _r);
    std::map<std::string, HSVRange> range_;
    std::map<std::string, cv::Scalar> colour_;

    void setImageSize(const cv::Size& _size);
    cv::Mat frame_, hsv_, in_range_;
    cv::Size capture_size_;

    HSVDetectionConfig config_;
    bool tracking_ = false;
    cv::Point2f tracking_target_;
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

inline void HSVDetection::setFrame(cv::Mat& _frame) {
    if (_frame.empty()) {
        ROS_WARN("HSVDetection::setFrame: frame is empty!");
        return;
    }
    setImageSize(_frame.size());
    cvtColor(_frame, hsv_, cv::COLOR_BGR2HSV);
    frame_ = _frame;
}

inline void HSVDetection::drawRotatedRect(const cv::RotatedRect& _r, int _index, cv::Scalar _colour) {
    char count_text[32];
    sprintf(count_text, "%d", _index);
    cv::circle(frame_, _r.center, 4, _colour, -1);
    cv::putText(frame_, count_text, _r.center, CV_FONT_HERSHEY_PLAIN, 1, _colour);
    cv::Point2f vertices[4];
    _r.points(vertices);
    for (int i = 0; i < 4; i++) {
        line(frame_, vertices[i], vertices[(i+1)%4], _colour, 2);
    }
}

// TODO: Not the most elegant solution...
std::vector<HSVItem> HSVDetection::detectPipeline(const std::string _id, const cv::Mat& _src, bool _draw) {

    if (range_.count(_id) == 0) {
        ROS_WARN("HSVDetection::detectPipeline: id [%s] not found!", _id.c_str());
        return std::vector<HSVItem>();
    }
    if (_src.empty()) {
        ROS_WARN("HSVDetection::detectPipeline: source is empty!");
        return std::vector<HSVItem>();
    }

    // Detect the object based on HSV Range Values
    cv::Mat in_range;
    if (range_[_id].min_HSV[0] >= range_[_id].max_HSV[0]) {
        HSVRange lo, hi;
        lo = range_[_id];
        lo.min_HSV[0] = 0;
        lo.max_HSV[0] = range_[_id].max_HSV[0];
        hi = range_[_id];
        hi.min_HSV[0] = range_[_id].min_HSV[0];
        hi.max_HSV[0] = MAX_VALUE_H;
        cv::Mat in_range_lo, in_range_hi;
        inRange(_src, lo.getMin(), lo.getMax(), in_range_lo);
        inRange(_src, hi.getMin(), hi.getMax(), in_range_hi);
        in_range = in_range_lo + in_range_hi;
    } else {
        inRange(_src, range_[_id].getMin(), range_[_id].getMax(), in_range);
    }

    // Smooth image...
    morphologyEx(in_range, in_range, cv::MORPH_OPEN, config_.kernel.getAsElement());

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(in_range, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    /// Draw contours and generate HSVItems
    int valid_index = 0;
    std::vector<HSVItem> item_list;
    for (int i = 0; i < contours.size(); i++) {

        cv::Mat polygon;
        cv::approxPolyDP(cv::Mat(contours[i]), polygon, config_.poly_epsilon, true);
        double area = fabs(cv::contourArea(polygon));
        if (area < config_.min_area) { continue; }

        cv::RotatedRect rect = sanitizeRotatedRect(minAreaRect(polygon));  // TODO: contours[i]? polygon?
        // CvMoments moments = cv::moments(polygon);
        // cv::Point centroid = cv::Point(cvRound(moments.m10/moments.m00), cvRound(moments.m01/moments.m00));
        // double mu20_prime = moments.mu20 / moments.m00;
        // double mu02_prime = moments.mu02 / moments.m00;
        // double mu11_prime = moments.mu11 / moments.m00;
        // double theta = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);

        if (_draw) {
            cv::Scalar colour = colour_[_id];
            CvMoments moments = cv::moments(polygon);
            cv::Point centroid = cv::Point(cvRound(moments.m10/moments.m00), cvRound(moments.m01/moments.m00));
            cv::circle(frame_, centroid, 4, colour, 1);
            cv::drawContours(frame_, contours, i, colour, 1);
            cv::drawContours(frame_, polygon, -1, colour, 3);
            drawRotatedRect(rect, valid_index++, colour);
        }

        HSVItem item;
        item.detector_id = _id;
        item.rectangle = rect;
        item.cropped = is_cropped(rect);
        item_list.push_back(item);
     }

    return item_list;   
}

inline std::vector<HSVItem> HSVDetection::detect(const std::string _id, bool _draw) {
    return detectPipeline(_id, hsv_, _draw);
}

inline std::vector<HSVItem> HSVDetection::detectAll(bool _draw) {
    std::vector<HSVItem> final_list;
    for (auto it = range_.begin(); it != range_.end(); ++it) {
          std::vector<HSVItem> partial_list = detect(it->first, _draw);
        final_list.insert(final_list.end(), partial_list.begin(), partial_list.end());
    }
    return final_list;
}

std::vector<HSVItem> HSVDetection::track(const std::string _id, bool _draw) {

    std::vector<HSVItem> detected = detect(_id, true);
    if (detected.size() == 0) { return std::vector<HSVItem>(); }

    cv::Size hsv_size = hsv_.size();
    if (!tracking_) {
        tracking_target_.x = hsv_size.width / 2.0;
        tracking_target_.y = hsv_size.height / 2.0;
        tracking_ = true;
    }

    std::vector<HSVItem> tracking_list;
    HSVItem closest_colour;
    float min_sq_distance = (hsv_size.width + hsv_size.width) * (hsv_size.width + hsv_size.width);
    for (auto item: detected) {
        float dx = fabs(tracking_target_.x - item.rectangle.center.x);
        float dy = fabs(tracking_target_.y - item.rectangle.center.y);
        float sq_distance = dx*dx + dy*dy;
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            closest_colour = item;
        }
    }
    tracking_list.push_back(closest_colour);
    tracking_target_ = closest_colour.rectangle.center;

    if (_draw) {
        rectangle(frame_, closest_colour.rectangle.boundingRect(), colour_[_id]);
    }

    return tracking_list;
}

HSVTrackingPair HSVDetection::trackBrick(const std::string _id, bool _draw) {

    std::vector<HSVItem> detected = detect(_id, true);
    if (detected.size() == 0) { return HSVTrackingPair(); }

    cv::Size hsv_size = hsv_.size();
    if (!tracking_) {
        tracking_target_.x = hsv_size.width / 2.0;
        tracking_target_.y = hsv_size.height / 2.0;
        tracking_ = true;
    }

    HSVTrackingPair tracking_pair;
    HSVItem closest_colour;
    float min_sq_distance = (hsv_size.width + hsv_size.width) * (hsv_size.width + hsv_size.width);
    for (auto item: detected) {
        float dx = fabs(tracking_target_.x - item.rectangle.center.x);
        float dy = fabs(tracking_target_.y - item.rectangle.center.y);
        float sq_distance = dx*dx + dy*dy;
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            closest_colour = item;
            tracking_pair.has_colour = true;
        }
    }
    tracking_pair.colour_item = closest_colour;
    tracking_target_ = closest_colour.rectangle.center;

    cv::Rect roi = closest_colour.rectangle.boundingRect() & cv::Rect(0, 0, hsv_size.width, hsv_size.height);
    if ((roi.width <= 0) || (roi.height <= 0)) { return tracking_pair; }  // TODO: Should be an error?
    // ROS_ERROR("(x, y) = (%d, %d), (w, h) = (%d, %d)", roi.x, roi.y, roi.width, roi.height);

    // range_["white"] is mandatory here!
    if (range_.count("white") == 0) {
        ROS_WARN("HSVDetection::track: id [white] not found!");
        return tracking_pair;
    }
    cv::Mat hsv_roi = hsv_(roi);
    std::vector<HSVItem> white_list = detectPipeline("white", hsv_roi, false);
    if (white_list.size() == 0) {
        // ROS_WARN("HSVDetection::track: white not found!");
        return tracking_pair;
    }

    // HSVItem largest_white;
    // float max_area = 0;
    // for (auto item: white_list) {
    //     float area = item.rectangle.size.width * item.rectangle.size.height;
    //     if (area > max_area) {
    //         max_area = area;
    //         largest_white = item;
    //     }
    // }
    // // Traslate largest_white out of ROI:
    // largest_white.rectangle.center.x += roi.x;
    // largest_white.rectangle.center.y += roi.y;
    // largest_white.cropped = is_cropped(largest_white.rectangle);
    // tracking_pair.white_item = largest_white;

    // Use closest white to ROI center instead of largest(orange!)
    cv::Size roi_size = hsv_roi.size();
    HSVItem closest_white;
    min_sq_distance = (hsv_size.width + hsv_size.width) * (hsv_size.width + hsv_size.width);
    for (auto item: white_list) {
        float dx = fabs(roi_size.width  / 2.0 - item.rectangle.center.x);
        float dy = fabs(roi_size.height / 2.0 - item.rectangle.center.y);
        float sq_distance = dx*dx + dy*dy;
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            closest_white = item;
            tracking_pair.has_white = true;
        }
    }
    // Traslate closest_white out of ROI:
    closest_white.rectangle.center.x += roi.x;
    closest_white.rectangle.center.y += roi.y;
    closest_white.cropped = is_cropped(closest_white.rectangle);
    tracking_pair.white_item = closest_white;

    cv::Point2f white_vertices[4];
    // largest_white.rectangle.points(white_vertices);
    closest_white.rectangle.points(white_vertices);
    float first_max  = 0;
    float second_max = 0;
    int first_max_index  = -1;
    int second_max_index = -1;
    for (int i = 0; i < 4 ; i ++) {
        if (white_vertices[i].y > first_max) {
            second_max = first_max;
            second_max_index = first_max_index;
            first_max = white_vertices[i].y;
            first_max_index = i;
        } else if (white_vertices[i].y > second_max) {
            second_max = white_vertices[i].y;
            second_max_index = i;
        }
    }
    if (first_max_index >= 0 && second_max_index >= 0) {
        tracking_pair.white_edge_center.x = 0.5 * (white_vertices[first_max_index].x + white_vertices[second_max_index].x);
        tracking_pair.white_edge_center.y = 0.5 * (white_vertices[first_max_index].y + white_vertices[second_max_index].y);
    }  // TODO: else?

    if (_draw) {
        rectangle(frame_, closest_colour.rectangle.boundingRect(), colour_[_id]);
        // drawRotatedRect(closest_colour.rectangle, 0, colour_[_id]);
        // drawRotatedRect(largest_white.rectangle, -1, colour_[_id]);
        drawRotatedRect(closest_white.rectangle, -1, colour_[_id]);
        cv::circle(frame_, tracking_pair.white_edge_center, 6, colour_[_id], 1);
    }

    return tracking_pair;
}

inline bool HSVDetection::is_cropped(const cv::RotatedRect& _r) {
    auto bb = _r.boundingRect();
    if ((bb.x <= 0) || (bb.y <= 0)) { return true; }
    if (((bb.x + bb.width) >= capture_size_.width) || ((bb.y + bb.height) >= capture_size_.height)) { return true; }
    return false;
}

#endif  // HSV_DETECTION_H
