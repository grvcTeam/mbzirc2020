#include <ros/ros.h>
// #include <ros/package.h>
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
  // tf2::Matrix3x3 R;
  // tf2::Vector3 T;
};

struct RealWorldRect {
  geometry_msgs::Point center;
  geometry_msgs::Vector3 size;
  double yaw;
};

// TODO: Camera utils class?
RealWorldRect fromCvRotatedRect(const cv::RotatedRect& _rect, const CameraParameters& _camera, double _estimated_z) {

  // TODO(Geolocation): We end using only _camera.R.transpose()...
  // const tf2::Matrix3x3 Rt = _camera.R.transpose();
  // tf2::Vector3 T_world = Rt * _camera.T;
  tf2::Vector3 K_0 = _camera.K.getRow(0);
  tf2::Vector3 K_1 = _camera.K.getRow(1);

  tf2::Vector3 ray;
  double aux = (_rect.center.y - K_1[2]) / K_1[1];
  ray[0] = (_rect.center.x - K_0[2] - K_0[1] * aux) / K_0[0];
  ray[1] = aux;
  ray[2] = 1.0;

  // tf2::Vector3 ray_world = Rt * ray;
  // if (ray_world[2] == 0) {
  //   ROS_WARN("geoLocate: ray_world[2] == 0");
  //   return RealWorldRect();
  // }

  tf2::Vector3 ray_world = ray;

  // mbzirc_comm_objs::ObjectDetection object;
  // object.header.frame_id = "map";  // TODO: arena? NO, it would affect relative measures too
  // object.header.stamp = ros::Time::now();
  // object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK_TRACK;

  RealWorldRect out;
  // The line equation is X = lambda * ray_world - T_world
  // lambda can be set because the z is known (estimated)
  double lambda = _estimated_z / ray_world[2];
  out.center.x = lambda * ray_world[0];
  out.center.y = lambda * ray_world[1];
  out.center.z = lambda * ray_world[2];

  double orientation = _rect.angle * M_PI / 180.0;  // Conversion to rad
  // tf2::Vector3 orientation_camera(cos(orientation), sin(orientation), 0);
  // tf2::Vector3 orientation_world = _camera.R * orientation_camera;
  // double theta_world = atan2(orientation_world[1], orientation_world[0]);
  // printf("orientation_world = [%lf, %lf, %lf]\n", orientation_world[0], orientation_world[1], orientation_world[2]);
  // printf("theta_world = %lf\n", theta_world);
  out.yaw = orientation;
  // object.pose.pose.orientation.x = 0;
  // object.pose.pose.orientation.y = 0;
  // object.pose.pose.orientation.z = sin(0.5*theta_world);
  // object.pose.pose.orientation.w = cos(0.5*theta_world);
  // object.pose.covariance[0] = 0.01;  // TODO: Covariance?
  // object.pose.covariance[7] = 0.01;
  // object.pose.covariance[14] = 0.01;

  // Suppose item is a rectangle: P = 2 * (side_1 + side_2); A = side_1 * side_2
  // side_i = (P ± sqrt(P*P - 16*A)) / 4
  double pixel_to_metric_x = lambda / K_0[0];
  double pixel_to_metric_y = lambda / K_1[1];
  out.size.x = pixel_to_metric_x * _rect.size.width;
  out.size.y = pixel_to_metric_y * _rect.size.height;
  out.size.z = 0.2;
  // out.color = color_from_string(item.detector_id);
  // object_list.objects.push_back(object);
  // std::cout << object << '\n';

  return out;
}

mbzirc_comm_objs::ObjectDetection fromHSVTrackingPair(const HSVTrackingPair& _hsv_tracking_pair, const CameraParameters& _camera, double _estimated_z) {

  // TODO: repeated!
  // const tf2::Matrix3x3 Rt = _camera.R.transpose();
  // tf2::Vector3 T_world = Rt * _camera.T;

  // double estimated_z = 0.5;  // Estimated vertical distance to bricks (TODO: use sf11)

  RealWorldRect rect_real;
  std::string tracked_color;
  if (!_hsv_tracking_pair.colour_item.cropped) {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.colour_item.rectangle, _camera, _estimated_z);
    tracked_color = _hsv_tracking_pair.colour_item.detector_id;
  } else if (!_hsv_tracking_pair.white_item.cropped) {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.white_item.rectangle, _camera, _estimated_z);
    tracked_color = _hsv_tracking_pair.white_item.detector_id;
  } else {
    rect_real = fromCvRotatedRect(_hsv_tracking_pair.white_item.rectangle, _camera, _estimated_z);
    tracked_color = _hsv_tracking_pair.white_item.detector_id;
    // TODO: estimate white center from edge position and color
  }

  mbzirc_comm_objs::ObjectDetectionList object_list;
  mbzirc_comm_objs::ObjectDetection object;
  object.header.frame_id = _camera.frame_id;
  object.header.stamp = ros::Time::now();
  object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK_TRACK;

  // object.relative_position = rect_real.center;
  // object.pose.header.frame_id = "camera_link";  // TODO: camera_optical_frame
  // object.pose.header.stamp = ros::Time::now();
  object.pose.pose.position = rect_real.center;

  // object.relative_yaw = rect_real.yaw;
  object.pose.pose.orientation.x = 0;
  object.pose.pose.orientation.y = 0;
  object.pose.pose.orientation.z = sin(0.5*rect_real.yaw);
  object.pose.pose.orientation.w = cos(0.5*rect_real.yaw);
  object.pose.covariance[0] = 0.01;
  object.pose.covariance[7] = 0.01;
  object.pose.covariance[14] = 0.01;

  object.scale = rect_real.size;
  object.color = color_from_string(tracked_color);
  // object_list.objects.push_back(object);
  // std::cout << object << '\n';
  // printf("pose: [%lf, %lf, %lf] (%lf)\n", object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, rect_real.yaw);

  return object;
}

// dummy service callback to make this node compatible with the fake camera plugin which allows to set the object types which can be recognized
bool ChangeTypesCB(mbzirc_comm_objs::DetectTypes::Request& req,
                          mbzirc_comm_objs::DetectTypes::Response &res)
{
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
  ros::param::param<std::string>("~tf_prefix", tf_prefix, "mbzirc");
  ros::param::param<std::string>("~camera_url", camera_url, "camera/color");
  // ros::param::param<std::string>("~camera_url", camera_url, "KINECT/camera_kinect/depth");

  std::string window_detection_name = "HSV Detection";
  ros::Publisher tracked_pub = nh.advertise<mbzirc_comm_objs::ObjectDetection>("tracked_object", 1);
  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "hsv_detection", true, window_detection_name);  // TODO: image_raw vs image_rect_color
  ros::ServiceServer types_server = nh.advertiseService("set_types", ChangeTypesCB);
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &sf11RangeCallback);

  HSVDetectionConfig detection_config;
  detection_config.min_area = 2;
  detection_config.poly_epsilon = 5;
  detection_config.kernel.type = 0;
  detection_config.kernel.size = 1;

  HSVDetection detection;
  detection.setConfig(detection_config);

  HSVRange red_range;
  red_range.min_HSV[0] = 0;
  red_range.max_HSV[0] = 17;
  red_range.min_HSV[1] = 112;
  red_range.max_HSV[1] = 255;
  red_range.min_HSV[2] = 98;
  red_range.max_HSV[2] = 255;
  detection.addDetector("red", red_range, cvScalar(0, 255, 0));

  HSVRange white_range;
  white_range.min_HSV[0] = 0;
  white_range.max_HSV[0] = 180;
  white_range.min_HSV[1] = 0;
  white_range.max_HSV[1] = 58;
  white_range.min_HSV[2] = 150;
  white_range.max_HSV[2] = 255;
  detection.addDetector("white", white_range, cvScalar(0, 0, 0));
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
  CameraParameters camera;
  camera.K = image_converter.getCameraK();
  // camera.frame_id = tf_prefix + "/camera_color_optical_frame";
  camera.frame_id = "camera_color_optical_frame";

  // tf2_ros::Buffer tf_buffer;
  // tf2_ros::TransformListener tf_listener(tf_buffer);
  // const tf2::Matrix3x3 link_to_cv(0,-1,0, 0,0,-1, 1,0,0);

  ros::Rate rate(10);  // [Hz] TODO: Tune!
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();

      // Pass frame to the hsv-model-based tracker:
      detection.setFrame(cv_ptr->image);

      // std::vector<HSVItem> detected = detection.detect("white", true);
      // std::vector<HSVItem> detected = detection.detectAll(true);
      // Print detected items:
      // for (int i = 0; i < detected.size(); i++) {
      //  	printf("[%d] Detected: centroid = {%d, %d}, area = %lf, detector = {%s}\n", i, detected[i].centroid.x, detected[i].centroid.y, detected[i].area, detected[i].detector_id.c_str());
      // }

      HSVTrackingPair tracked = detection.track("red", true);

      draw_hud(cv_ptr);
      image_converter.publish(cv_ptr);  // TODO: Optional!

      if (tracked.is_valid) {
        // tracked.print();
        double estimated_z = sf11_range.range - DELTA_H - 0.2;  // TODO: tf? CHECK!
        tracked_pub.publish(fromHSVTrackingPair(tracked, camera, estimated_z));  // TODO: Also detected!
      }

      // try {
      //   geometry_msgs::TransformStamped camera_link_tf = tf_buffer.lookupTransform("camera_link", "map", ros::Time(0));
      //   tf2::Stamped<tf2::Transform> camera_link_tf2;
      //   tf2::fromMsg(camera_link_tf, camera_link_tf2);
      //   camera.R = link_to_cv * camera_link_tf2.getBasis();
      //   camera.T = link_to_cv * camera_link_tf2.getOrigin();
      //   if (tracked.is_valid) {
      //     tracked.print();
      //     sensed_pub.publish(fromHSVTrackingPair(tracked, camera));  // TODO: Also detected!
      //   }
      // } catch (tf2::TransformException &e) {
      //   ROS_WARN("%s", e.what());
      //   continue;
      // }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
