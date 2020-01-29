#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter {
public:

  // TODO(performance): Another constructor that takes camera_info from static data or config file
  ImageConverter(const std::string& _info_topic, const std::string& _input_topic, const std::string& _output_topic, \
                 bool _use_gui = false, const std::string& _gui_window_name = ""): it_(nh_), use_gui_(_use_gui) {
    info_sub_ = nh_.subscribe(_info_topic, 1, &ImageConverter::infoCallback, this);
    image_sub_ = it_.subscribe(_input_topic, 1, &ImageConverter::imageCallback, this);
    image_pub_ = it_.advertise(_output_topic, 1);
    if (use_gui_) {
      gui_window_name_ = (_gui_window_name != "")? _gui_window_name: _output_topic;
      cv::namedWindow(gui_window_name_);
    }
  }

  ~ImageConverter() {
    if (use_gui_) { cv::destroyWindow(gui_window_name_); }
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info_ = *msg;
    has_camera_info_ = true;
  }

  bool hasCameraInfo() { return has_camera_info_; }

  tf2::Matrix3x3 getCameraK() {
    return tf2::Matrix3x3(camera_info_.K[0], camera_info_.K[1], camera_info_.K[2], 
                          camera_info_.K[3], camera_info_.K[4], camera_info_.K[5],
                          camera_info_.K[6], camera_info_.K[7], camera_info_.K[8]);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      has_new_image_ = true;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  bool hasNewImage() { return has_new_image_; }

  cv_bridge::CvImagePtr getCvImagePtr() {
    has_new_image_ = false;
    return cv_ptr_;
  }

  void publish(cv_bridge::CvImagePtr _ptr) {
    if (!_ptr || _ptr->image.empty()) {
      return;
    }

    if (use_gui_) {
      cv::imshow(gui_window_name_, _ptr->image);
      cv::waitKey(3);
    }
    image_pub_.publish(_ptr->toImageMsg());
  }

protected:

  ros::NodeHandle nh_;
  ros::Subscriber info_sub_;
  sensor_msgs::CameraInfo camera_info_;
  bool has_camera_info_ = false;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_ptr_ = nullptr;
  bool has_new_image_ = false;

  bool use_gui_;
  std::string gui_window_name_;
};

#endif  // IMAGE_CONVERTER_H
