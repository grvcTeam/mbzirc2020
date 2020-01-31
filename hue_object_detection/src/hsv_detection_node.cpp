#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hue_object_detection/ImageConverter.h>
#include <hue_object_detection/HSVDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/DetectTypes.h>

#define DELTA_H 0.1  // TODO!

// TODO: Move to some kind of utils lib, as it is repeated
int color_from_string(const std::string& color) {
    int out_color;
    switch(color[0]) {
        case 'R':
        case 'r':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_RED;
            break;
        case 'G':
        case 'g':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_GREEN;
            break;
        case 'B':
        case 'b':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_BLUE;
            break;
        case 'O':
        case 'o':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_ORANGE;
            break;
        case 'W':
        case 'w':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_WHITE;
            break;
        default:
        ROS_ERROR("Unknown color %s", color.c_str());
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
    }
    return out_color;
}

// TODO: to header?
struct CameraParameters {
  std::string frame_id;
  tf2::Matrix3x3 K;
};

struct RealWorldRect {
  geometry_msgs::Point center;
  geometry_msgs::Vector3 size;
  double yaw;
};

// TODO: Camera utils class?
RealWorldRect fromCvRotatedRect(const cv::RotatedRect& _rect, const CameraParameters& _camera, double _estimated_z) {

  tf2::Vector3 K_0 = _camera.K.getRow(0);
  tf2::Vector3 K_1 = _camera.K.getRow(1);

  tf2::Vector3 ray;
  double aux = (_rect.center.y - K_1[2]) / K_1[1];
  ray[0] = (_rect.center.x - K_0[2] - K_0[1] * aux) / K_0[0];
  ray[1] = aux;
  ray[2] = 1.0;

  RealWorldRect out;
  // The line equation is X = lambda * ray_world
  // lambda can be set because the z is known (estimated)
  double lambda = _estimated_z / ray[2];
  out.center.x = lambda * ray[0];
  out.center.y = lambda * ray[1];
  out.center.z = lambda * ray[2];
  out.yaw = _rect.angle * M_PI / 180.0;  // Conversion to rad

  double pixel_to_metric_x = lambda / K_0[0];
  double pixel_to_metric_y = lambda / K_1[1];
  out.size.x = pixel_to_metric_x * _rect.size.width;  // TODO: as a function of color?
  out.size.y = pixel_to_metric_y * _rect.size.height;
  out.size.z = 0.2;

  return out;
}

// TODO: Repeated code...
geometry_msgs::Point fromCvPoint(const cv::Point2f& _point, const CameraParameters& _camera, double _estimated_z) {

  tf2::Vector3 K_0 = _camera.K.getRow(0);
  tf2::Vector3 K_1 = _camera.K.getRow(1);

  tf2::Vector3 ray;
  double aux = (_point.y - K_1[2]) / K_1[1];
  ray[0] = (_point.x - K_0[2] - K_0[1] * aux) / K_0[0];
  ray[1] = aux;
  ray[2] = 1.0;

  geometry_msgs::Point out;
  // The line equation is X = lambda * ray_world
  // lambda can be set because the z is known (estimated)
  double lambda = _estimated_z / ray[2];
  out.x = lambda * ray[0];
  out.y = lambda * ray[1];
  out.z = lambda * ray[2];

  return out;
}

float whiteHalfHeight(const std::string& _color) {
  float white_half_height;
  switch (_color[0]) {
  case 'r':
    white_half_height = 0.1;
    break;
  case 'g':
    white_half_height = 0.15;
    break;
  case 'b':
    white_half_height = 0.15;
    break;
  case 'o':
    white_half_height = 0.095;
    break;
  default:
    white_half_height = 0;
    ROS_WARN("Unexpected color: %s", _color.c_str());

  }
  return white_half_height;
}

mbzirc_comm_objs::ObjectDetection fromHSVTrackingPair(const HSVTrackingPair& _hsv_tracking_pair, const CameraParameters& _camera, double _estimated_z) {

  RealWorldRect rect_real;
  std::string tracked_color;
  bool is_cropped = false;
  geometry_msgs::Point white_edge_center = fromCvPoint(_hsv_tracking_pair.white_edge_center, _camera, _estimated_z);
  if (!_hsv_tracking_pair.colour_item.cropped) {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.colour_item.rectangle, _camera, _estimated_z);
    tracked_color = _hsv_tracking_pair.colour_item.detector_id;
  } else if (!_hsv_tracking_pair.white_item.cropped) {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.white_item.rectangle, _camera, _estimated_z);
    tracked_color = _hsv_tracking_pair.white_item.detector_id;
  } else {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.white_item.rectangle, _camera, _estimated_z);
    rect_real.center.x = white_edge_center.x;
    rect_real.center.y = white_edge_center.y - whiteHalfHeight(_hsv_tracking_pair.colour_item.detector_id);  // camera rotation!
    tracked_color = _hsv_tracking_pair.white_item.detector_id;
    is_cropped = true;
  }

  mbzirc_comm_objs::ObjectDetection object;
  object.header.frame_id = _camera.frame_id;
  object.header.stamp = ros::Time::now();
  object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK_TRACK;

  object.pose.pose.position = rect_real.center;
  object.pose.pose.orientation.x = 0;
  object.pose.pose.orientation.y = 0;
  object.pose.pose.orientation.z = sin(0.5*rect_real.yaw);
  object.pose.pose.orientation.w = cos(0.5*rect_real.yaw);
  object.pose.covariance[0] = 0.01;
  object.pose.covariance[7] = 0.01;
  object.pose.covariance[14] = 0.01;

  object.point_of_interest = white_edge_center;
  object.scale = rect_real.size;  // TODO: As a function of color
  object.color = color_from_string(tracked_color);
  object.is_cropped = is_cropped;
  // object.is_cropped = _hsv_tracking_pair.colour_item.cropped;

  return object;
}

mbzirc_comm_objs::ObjectDetectionList fromHSVItemList(const std::vector<HSVItem>& _hsv_item_list, const CameraParameters& _camera, double _estimated_z) {

  mbzirc_comm_objs::ObjectDetectionList object_list;

  for (auto item: _hsv_item_list) {
    RealWorldRect rect_real = fromCvRotatedRect(item.rectangle, _camera, _estimated_z);

    mbzirc_comm_objs::ObjectDetection object;
    object.header.frame_id = _camera.frame_id;
    object.header.stamp = ros::Time::now();
    object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK;

    object.pose.pose.position = rect_real.center;
    object.pose.pose.orientation.x = 0;
    object.pose.pose.orientation.y = 0;
    object.pose.pose.orientation.z = sin(0.5*rect_real.yaw);
    object.pose.pose.orientation.w = cos(0.5*rect_real.yaw);
    object.pose.covariance[0] = 0.01;
    object.pose.covariance[7] = 0.01;
    object.pose.covariance[14] = 0.01;

    object.scale = rect_real.size;  // TODO: from color
    object.color = color_from_string(item.detector_id);
    object.is_cropped = item.cropped;
    object_list.objects.push_back(object);
    // std::cout << object << '\n';
  }

  return object_list;
}

mbzirc_comm_objs::DetectTypes::Request g_detect_request;  // TODO: Global!
bool ChangeTypesCB(mbzirc_comm_objs::DetectTypes::Request& req,
                          mbzirc_comm_objs::DetectTypes::Response &res)
{
  g_detect_request = req;
  res.success = true;
  return true;
}

void draw_hud(cv_bridge::CvImagePtr _ptr) {
  if (!_ptr || _ptr->image.empty()) {
    return;
  }
  cv::Size image_size = _ptr->image.size();
  int half_height = cvRound(image_size.height / 2);
  int half_width = cvRound(image_size.width / 2);
  cv::Scalar color = cvScalar(0, 255, 255);
  cv::line(_ptr->image, cvPoint(0, half_height), cvPoint(image_size.width, half_height), color);
  cv::line(_ptr->image, cvPoint(half_width, 0), cvPoint(half_width, image_size.height), color);
}

sensor_msgs::Range sf11_range;
void sf11RangeCallback(const sensor_msgs::RangeConstPtr& msg) {
  sf11_range = *msg;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "hsv_detection_node");
  ros::NodeHandle nh;

  std::string tf_prefix;
  std::string camera_url;
  ros::param::param<std::string>("~tf_prefix", tf_prefix, "default_prefix");
  ros::param::param<std::string>("~camera_url", camera_url, "camera/color");

  ros::Publisher sensed_pub = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1);
  ros::Publisher tracked_pub = nh.advertise<mbzirc_comm_objs::ObjectDetection>("tracked_object", 1);
  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "hsv_detection", false, "HSV Detection");  // TODO: image_raw vs image_rect_color
  ros::ServiceServer types_server = nh.advertiseService("set_types", ChangeTypesCB);
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &sf11RangeCallback);

  std::string config_folder = ros::package::getPath("hue_object_detection") + "/config/";
  std::string config_filename = config_folder + "hsv_config.yaml";  // TODO: From param?

  YAML::Node config_yaml = YAML::LoadFile(config_filename);
  std::map<std::string, HSVRange> range_map;
  config_yaml["colors"]["red"]    >> range_map["red"];
  config_yaml["colors"]["green"]  >> range_map["green"];
  config_yaml["colors"]["blue"]   >> range_map["blue"];
  config_yaml["colors"]["orange"] >> range_map["orange"];
  config_yaml["colors"]["white"]  >> range_map["white"];

  HSVDetectionConfig detection_config;
  config_yaml["detection_config"] >> detection_config;
  // printf("config: %lf, %lf, %d, %d\n", detection_config.min_area, detection_config.poly_epsilon, detection_config.kernel.type, detection_config.kernel.size);

  HSVDetection detection;
  detection.setConfig(detection_config);
  detection.addDetector("red", range_map["red"], cvScalar(0, 255, 0));
  detection.addDetector("green", range_map["green"], cvScalar(255, 0, 0));
  detection.addDetector("blue", range_map["blue"], cvScalar(255, 255, 0));
  detection.addDetector("orange", range_map["orange"], cvScalar(0, 255, 255));
  detection.addDetector("white", range_map["white"], cvScalar(0, 0, 0));

  while (!image_converter.hasCameraInfo() && ros::ok()) {
    // TODO(performance): If camera info is constant, having a subscriber is overkill
    ROS_INFO("Waiting for camera info...");
    ros::spinOnce();
    sleep(1);
  }
  CameraParameters camera;
  camera.K = image_converter.getCameraK();
  camera.frame_id = tf_prefix + "/camera_color_optical_frame";

  ros::Rate rate(20);  // [Hz] TODO: Tune!
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();

      // Pass frame to the hsv-model-based tracker:
      detection.setFrame(cv_ptr->image);
      double estimated_z = sf11_range.range - DELTA_H - 0.2;  // TODO: tf? CHECK!

      switch (g_detect_request.command) {
        case mbzirc_comm_objs::DetectTypes::Request::COMMAND_DETECT_ALL:
        {
          std::vector<HSVItem> detected = detection.detectAll(true);
          sensed_pub.publish(fromHSVItemList(detected, camera, estimated_z));
          // Print detected items:
          // for (int i = 0; i < detected.size(); i++) {
          //  	printf("[%d] Detected: centroid = {%d, %d}, area = %lf, detector = {%s}\n", i, detected[i].centroid.x, detected[i].centroid.y, detected[i].area, detected[i].detector_id.c_str());
          // }
          break;
        }

        case mbzirc_comm_objs::DetectTypes::Request::COMMAND_TRACK_BRICK:
        {
          if (g_detect_request.types.size() <= 0) {
            ROS_ERROR("types is empty, nothing to track!");
          }
          std::string target = g_detect_request.types[0];  // We track first element
          HSVTrackingPair tracked = detection.track(target, true);
          if (tracked.has_colour && tracked.has_white) {
            // tracked.print();
            tracked_pub.publish(fromHSVTrackingPair(tracked, camera, estimated_z));
          }
          break;
        }

        case mbzirc_comm_objs::DetectTypes::Request::COMMAND_STOP_TRACKING:
        {
          detection.stopTracking();
          break;
        }

      }

      if (g_detect_request.visualize) {
        draw_hud(cv_ptr);
        image_converter.publish(cv_ptr);
      }

    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
