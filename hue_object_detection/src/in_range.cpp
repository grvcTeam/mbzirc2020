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
#include <hue_object_detection/HSVDetection.h>
// #include <mbzirc_comm_objs/ObjectDetectionList.h>
// #include <mbzirc_comm_objs/DetectTypes.h>

// Globals:
std::string window_detection_name = "Object Detection";
HSVRange range;

static void on_min_H_thresh_trackbar(int, void *) {
  range.min_HSV[0] = cv::min(range.max_HSV[0]-1, range.min_HSV[0]);
  cv::setTrackbarPos("Min H", window_detection_name, range.min_HSV[0]);
}

static void on_max_H_thresh_trackbar(int, void *) {
  range.max_HSV[0] = cv::max(range.max_HSV[0], range.min_HSV[0]+1);
  cv::setTrackbarPos("Max H", window_detection_name, range.max_HSV[0]);
}

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

  ros::init(argc, argv, "hsv_detection_node");
  ros::NodeHandle nh;

  std::string tf_prefix;
  std::string camera_url;
  ros::param::param<std::string>("~tf_prefix", tf_prefix, "mbzirc");
  ros::param::param<std::string>("~camera_url", camera_url, "camera/color");

//   ros::Publisher sensed_pub = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 10);
  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "hsv_detection", true, window_detection_name);  // TODO: image_raw vs image_rect_color
//   ros::ServiceServer types_server = nh.advertiseService("set_types", ChangeTypesCB);

  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Min H", window_detection_name, &range.min_HSV[0], MAX_VALUE_H, on_min_H_thresh_trackbar);
  cv::createTrackbar("Max H", window_detection_name, &range.max_HSV[0], MAX_VALUE_H, on_max_H_thresh_trackbar);
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
//   detection.setConfig(detection_config);
//   std::string config_folder = ros::package::getPath("hue_object_detection") + "/config/";
//   detection.addDetector("red", red_range, cvScalar(255, 0, 0));
//   detection.addDetector("green", green_range, cvScalar(255, 0, 255));
//   detection.addDetector("blue", blue_range, cvScalar(0, 255, 255));
//   detection.addDetector("orange", blue_range, cvScalar(255, 0, 0));

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

  // cv::Mat frame_HSV, frame_threshold;

  ros::Rate rate(20);  // [Hz] TODO: Tune!
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();

      // cvtColor(cv_ptr->image, frame_HSV, cv::COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      // inRange(frame_HSV, cv::Scalar(range.min_HSV[0], range.min_HSV[1], range.min_HSV[2]), cv::Scalar(range.max_HSV[0], range.max_HSV[1], range.max_HSV[2]), frame_threshold);

      // Pass frame to the hsv-model-based tracker:
      detection.addDetector("test", range, cvScalar(0, 255, 0));
      detection_config.min_area = min_area;
      detection_config.poly_epsilon = poly_epsilon;
      detection.setConfig(detection_config);
      detection.setFrame(cv_ptr->image);
      // detection.detect("test", true);
      // detection.detect("white", true);
      // detection.detectAll(true);
      std::vector<HSVItem> tracked = detection.track("test", true);
      for (int i = 0; i < tracked.size(); i++) {
        auto bb = tracked[i].rectangle.boundingRect();
       	printf("[%d] Tracking[%s]: (%f, %f), [%f, %f], %fº%s\n", i, tracked[i].detector_id.c_str(), tracked[i].rectangle.center.x, tracked[i].rectangle.center.y, tracked[i].rectangle.size.width, tracked[i].rectangle.size.height, tracked[i].rectangle.angle, tracked[i].cropped? ", cropped!": "");
        // printf("[%s]: bb.x + bb.width = %d, bb.y + bb.height = %d\n", tracked[i].detector_id.c_str(), bb.x + bb.width, bb.y + bb.height);
      }

      //cv_ptr->image = detection.getDetection();  // debug!

    //   std::vector<HSVItem> detected = detection.detectAll(true);
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
    //     sensed_pub.publish(fromHSVItem(detected, camera));

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
