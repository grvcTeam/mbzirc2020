#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hue_object_detection/ImageConverter.h>
#include <hue_object_detection/HSVDetection.h>

// Globals:
std::string window_detection_name = "Object Detection";
HSVRange range;

static void on_min_S_thresh_trackbar(int, void *) {
  range.min_HSV[1] = cv::min(range.max_HSV[1]-1, range.min_HSV[1]);
  cv::setTrackbarPos("Min S", window_detection_name, range.min_HSV[1]);
}

static void on_max_S_thresh_trackbar(int, void *) {
  range.max_HSV[1] = cv::max(range.max_HSV[1], range.min_HSV[1]+1);
  cv::setTrackbarPos("Max S", window_detection_name, range.max_HSV[1]);
}

static void on_min_V_thresh_trackbar(int, void *) {
  range.min_HSV[2] = cv::min(range.max_HSV[2]-1, range.min_HSV[2]);
  cv::setTrackbarPos("Min V", window_detection_name, range.min_HSV[2]);
}

static void on_max_V_thresh_trackbar(int, void *) {
  range.max_HSV[2] = cv::max(range.max_HSV[2], range.min_HSV[2]+1);
  cv::setTrackbarPos("Max V", window_detection_name, range.max_HSV[2]);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "hsv_calibration");
  ros::NodeHandle nh;

  std::string camera_url;
  ros::param::param<std::string>("~camera_url", camera_url, "camera/color");  // camera_0 (sim), camera/color (real)
  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "hsv_detection", true, window_detection_name);  // TODO: image_raw vs image_rect_color

  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Min H*", window_detection_name, &range.min_HSV[0], MAX_VALUE_H);
  cv::createTrackbar("Max H*", window_detection_name, &range.max_HSV[0], MAX_VALUE_H);
  cv::createTrackbar("Min S", window_detection_name, &range.min_HSV[1], MAX_VALUE_S, on_min_S_thresh_trackbar);
  cv::createTrackbar("Max S", window_detection_name, &range.max_HSV[1], MAX_VALUE_S, on_max_S_thresh_trackbar);
  cv::createTrackbar("Min V", window_detection_name, &range.min_HSV[2], MAX_VALUE_V, on_min_V_thresh_trackbar);
  cv::createTrackbar("Max V", window_detection_name, &range.max_HSV[2], MAX_VALUE_V, on_max_V_thresh_trackbar);

  HSVDetectionConfig detection_config;
  int min_area = std::round(detection_config.min_area);
  int poly_epsilon = std::round(detection_config.poly_epsilon);

  /// Create Kernel trackbar
  cv::createTrackbar("K-type: 0:R 1:C 2:E", window_detection_name, &detection_config.kernel.type, KERNEL_MAX_TYPE_COUNT);
  cv::createTrackbar("K-size: 2n+1", window_detection_name, &detection_config.kernel.size, KERNEL_MAX_SIZE);

  /// Create min_area and poly_epsilon trackbars
  cv::createTrackbar("Min area", window_detection_name, &min_area, 306081/2);  // TODO: Depends on resolution
  cv::createTrackbar("Epsilon", window_detection_name, &poly_epsilon, 640/4);  // TODO: Depends on resolution

  HSVDetection detection;
  HSVRange white_range;
  white_range.min_HSV[0] = 0;
  white_range.min_HSV[1] = 0;
  white_range.min_HSV[2] = 150;
  white_range.max_HSV[0] = 180;
  white_range.max_HSV[1] = 58;
  white_range.max_HSV[2] = 255;
  detection.addDetector("white", white_range, cvScalar(0, 0, 0));

  while (!image_converter.hasCameraInfo() && ros::ok()) {
    // TODO(performance): If camera info is constant, having a subscriber is overkill
    ROS_INFO("Waiting for camera info...");
    ros::spinOnce();
    sleep(1);
  }

  ros::Rate rate(20);  // [Hz] TODO: Tune!
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();

      // Pass frame to the hsv-model-based tracker:
      detection.addDetector("test", range, cvScalar(0, 255, 0));
      detection_config.min_area = min_area;
      detection_config.poly_epsilon = poly_epsilon;
      detection.setConfig(detection_config);
      detection.setFrame(cv_ptr->image);
      // detection.detect("test", true);
      // detection.detect("white", true);
      // detection.detectAll(true);
      HSVTrackingPair tracked = detection.track("test", true);
      tracked.print();
      // printf("[colour] orientation = %lf\n", tracked.colour_item.orientation*180/M_PI - 90);
      // printf("[white] orientation = %lf\n", tracked.white_item.orientation*180/M_PI - 90);

      // draw_hud(cv_ptr);
      image_converter.publish(cv_ptr);  // TODO: Optional!
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
