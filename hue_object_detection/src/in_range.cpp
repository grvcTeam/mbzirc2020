#include <ros/ros.h>
// #include <ros/package.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hue_object_detection/ImageConverter.h>
// #include <hue_object_detection/HueDetection.h>
// #include <mbzirc_comm_objs/ObjectDetectionList.h>
// #include <mbzirc_comm_objs/DetectTypes.h>

#define MAX_VALUE_H 180  // 180 = 360/2
#define MAX_VALUE_S 255
#define MAX_VALUE_V 255

/** Global Variables */
std::string window_detection_name = "Object Detection";
int low_H = 0;
int low_S = 0;
int low_V = 0;
int high_H = MAX_VALUE_H;
int high_S = MAX_VALUE_S;
int high_V = MAX_VALUE_V;

static void on_low_H_thresh_trackbar(int, void *) {
  low_H = cv::min(high_H-1, low_H);
  cv::setTrackbarPos("Low H", window_detection_name, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
  high_H = cv::max(high_H, low_H+1);
  cv::setTrackbarPos("High H", window_detection_name, high_H);
}

static void on_low_S_thresh_trackbar(int, void *) {
  low_S = cv::min(high_S-1, low_S);
  cv::setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
  high_S = cv::max(high_S, low_S+1);
  cv::setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
  low_V = cv::min(high_V-1, low_V);
  cv::setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
  high_V = cv::max(high_V, low_V+1);
  cv::setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "hsv_detection_node");
  ros::NodeHandle nh;

  std::string tf_prefix;
  std::string camera_url;
  ros::param::param<std::string>("~tf_prefix", tf_prefix, "mbzirc");
  ros::param::param<std::string>("~camera_url", camera_url, "camera/color");

//   ros::Publisher sensed_pub = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 10);
  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "hsv_detection", true, window_detection_name);
//   ros::ServiceServer types_server = nh.advertiseService("set_types", ChangeTypesCB);

  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Low H", window_detection_name, &low_H, MAX_VALUE_H, on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", window_detection_name, &high_H, MAX_VALUE_H, on_high_H_thresh_trackbar);
  cv::createTrackbar("Low S", window_detection_name, &low_S, MAX_VALUE_S, on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", window_detection_name, &high_S, MAX_VALUE_S, on_high_S_thresh_trackbar);
  cv::createTrackbar("Low V", window_detection_name, &low_V, MAX_VALUE_V, on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", window_detection_name, &high_V, MAX_VALUE_V, on_high_V_thresh_trackbar);

//   HueDetection detection;
//   HueDetectionConfig detection_config;
//   detection_config.saturation_threshold = 128.0;
//   detection_config.likelihood_threshold = 96.0;
//   detection_config.min_area = 2.0;
//   detection_config.poly_epsilon = 3.0;
//   detection.setConfig(detection_config);
//   std::string histogram_folder = ros::package::getPath("hue_object_detection") + "/config/";
//   detection.addDetector("red", histogram_folder + "red.yaml", cvScalar(255, 255, 0));
//   detection.addDetector("green", histogram_folder + "green.yaml", cvScalar(255, 0, 255));
//   detection.addDetector("blue", histogram_folder + "blue.yaml", cvScalar(0, 255, 255));
//   detection.addDetector("orange", histogram_folder + "orange.yaml", cvScalar(255, 0, 0));

  while (!image_converter.hasCameraInfo() && ros::ok()) {
    // TODO(performance): If camera info is constant, having a subscriber is overkill
    ROS_INFO("Waiting for camera info...");
    ros::spinOnce();
    sleep(1);
  }
//   CameraParameters camera;
//   camera.K = image_converter.getCameraK();

//   tf2_ros::Buffer tf_buffer;
//   tf2_ros::TransformListener tf_listener(tf_buffer);
//   const tf2::Matrix3x3 link_to_cv(0,-1,0, 0,0,-1, 1,0,0);

  cv::Mat frame_HSV, frame_threshold;

  ros::Rate rate(10);  // [Hz] TODO: Tune!
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();
      cvtColor(cv_ptr->image, frame_HSV, cv::COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
      cv_ptr->image = frame_threshold;

      // Pass frame to the hue-model-based tracker:
    //   detection.setFrame(cv_ptr->image);
    //   std::vector<HueItem> detected = detection.detectAll(true);
      // Print detected items:
      // for (int i = 0; i < detected.size(); i++) {
      //  	printf("[%d] Detected: centroid = {%d, %d}, area = %lf, detector = {%s}\n", i, detected[i].centroid.x, detected[i].centroid.y, detected[i].area, detected[i].detector_id.c_str());
      // }
    //   draw_hud(cv_ptr);
      image_converter.publish(cv_ptr);  // TODO: Optional!

    //   try {
    //     geometry_msgs::TransformStamped camera_link_tf = tf_buffer.lookupTransform(tf_prefix + "/camera_link", "map", ros::Time(0));
    //     tf2::Stamped<tf2::Transform> camera_link_tf2;
    //     tf2::fromMsg(camera_link_tf, camera_link_tf2);
    //     camera.R = link_to_cv * camera_link_tf2.getBasis();
    //     camera.T = link_to_cv * camera_link_tf2.getOrigin();
    //     sensed_pub.publish(fromHueItem(detected, camera));

    //   } catch (tf2::TransformException &e) {
    //     ROS_WARN("%s", e.what());
    //     continue;
    //   }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
