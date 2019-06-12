#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hue_object_detection/HueDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

class ImageConverter {
public:

  ImageConverter(const std::string& _input_topic, const std::string& _output_topic, bool _use_gui = false): it_(nh_), use_gui_(_use_gui) {
    image_sub_ = it_.subscribe(_input_topic, 1, &ImageConverter::imageCallback, this);
    image_pub_ = it_.advertise(_output_topic, 1);
    if (use_gui_) {
      gui_window_name_ = _output_topic;
      cv::namedWindow(gui_window_name_);
    }
  }

  ~ImageConverter() {
    if (use_gui_) { cv::destroyWindow(gui_window_name_); }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      has_new_data_ = true;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  bool hasNewData() { return has_new_data_; }

  cv_bridge::CvImagePtr getCvImagePtr() {
    has_new_data_ = false;
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
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_ptr_ = nullptr;
  bool has_new_data_ = false;

  bool use_gui_;
  std::string gui_window_name_;
};

struct CameraParameters {
  tf2::Matrix3x3 K;
  tf2::Matrix3x3 R;
  tf2::Vector3 T;
};

mbzirc_comm_objs::ObjectDetectionList fromHueItem(const std::vector<HueItem>& _hue_item_list, const CameraParameters& _camera) {

  mbzirc_comm_objs::ObjectDetectionList object_list;

  // TODO(Geolocation): We end using only _camera.R.transpose()...
  const tf2::Matrix3x3 Rt = _camera.R.transpose();
  tf2::Vector3 T_world = Rt * _camera.T;
  tf2::Vector3 K_0 = _camera.K.getRow(0);
  tf2::Vector3 K_1 = _camera.K.getRow(1);
  double estimated_z = 0.2;  // Estimated height of bricks (TODO: assure they are not stacked!)

  for (auto item: _hue_item_list) {
    // Geolocation:
    tf2::Vector3 ray;
	  double aux = (item.centroid.y - K_1[2]) / K_1[1];
    ray[0] = (item.centroid.x - K_0[2] - K_0[1] * aux) / K_0[0];
    ray[1] = aux;
    ray[2] = 1.0;

    tf2::Vector3 ray_world = Rt * ray;
    if (ray_world[2] == 0) {
      ROS_WARN("geoLocate: ray_world[2] == 0");
      continue;
    }

    mbzirc_comm_objs::ObjectDetection object;
    object.header.frame_id = "map";
    object.header.stamp = ros::Time::now();
    object.type = "brick";

    // The line equation is X = lambda * ray_world - T_world
    // lambda can be set because the z is known (estimated)
    double lambda = (estimated_z + T_world[2]) / ray_world[2];
    object.pose.pose.position.x = lambda * ray_world[0] - T_world[0];
    object.pose.pose.position.y = lambda * ray_world[1] - T_world[1];
    object.pose.pose.position.z = lambda * ray_world[2] - T_world[2];

    tf2::Vector3 orientation_camera(cos(item.orientation), sin(item.orientation), 0);
    tf2::Vector3 orientation_world = _camera.R * orientation_camera;
    double theta_world = atan2(orientation_world[1], orientation_world[0]);
    // printf("orientation_world = [%lf, %lf, %lf]\n", orientation_world[0], orientation_world[1], orientation_world[2]);
    // printf("theta_world = %lf\n", theta_world);
    object.pose.pose.orientation.x = 0;
    object.pose.pose.orientation.y = 0;
    object.pose.pose.orientation.z = sin(0.5*theta_world);
    object.pose.pose.orientation.w = cos(0.5*theta_world);
    object.pose.covariance[0] = 0.01;  // TODO: Covariance?
    object.pose.covariance[7] = 0.01;
    object.pose.covariance[14] = 0.01;

    // Suppose item is a rectangle: P = 2 * (side_1 + side_2); A = side_1 * side_2
    // side_i = (P ± sqrt(P*P - 16*A)) / 4
    aux = sqrt(item.perimeter * item.perimeter - 16.0 * item.area);
    double pixel_to_metric_x = lambda / K_0[0];
    double pixel_to_metric_y = lambda / K_1[1];
    object.scale.x = pixel_to_metric_x * (item.perimeter + aux) / 4.0;
    object.scale.y = pixel_to_metric_y * (item.perimeter - aux) / 4.0;
    object.scale.z = estimated_z;
    object.properties = "{color: \'" + item.detector_id + "\'}";
    object_list.objects.push_back(object);
    // std::cout << object << '\n';
  }
  return object_list;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "detection_node");
  ros::NodeHandle nh;

  std::string tf_prefix = "";  // TODO: Default value?
  if (ros::param::has("tf_prefix")) {
    ros::param::get("tf_prefix", tf_prefix);
  }

  ros::Publisher sensed_pub = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 10);
  ImageConverter image_converter("camera_0/image_raw", "hue_detection", true);

  HueDetection detection;
  std::string histogram_folder = ros::package::getPath("hue_object_detection") + "/config/";
  detection.addDetector("red", histogram_folder + "red.yaml", cvScalar(255, 255, 0));
  detection.addDetector("green", histogram_folder + "green.yaml", cvScalar(255, 0, 255));
  detection.addDetector("blue", histogram_folder + "blue.yaml", cvScalar(0, 255, 255));
  detection.addDetector("orange", histogram_folder + "orange.yaml", cvScalar(255, 0, 0));

  // TODO: from camera_info topic at ImageConverter?
  double fx = 674;
  double fy = 674;
  double u0 = 400;
  double v0 = 300;
  double gamma = 0;
  CameraParameters camera;
  camera.K = tf2::Matrix3x3(fx,gamma,u0, 0,fy,v0, 0,0,1);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  const tf2::Matrix3x3 link_to_cv(0,-1,0, 0,0,-1, 1,0,0);

  ros::Rate rate(10);  // [Hz]
  while (ros::ok()) {
    if (image_converter.hasNewData()) {
      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();
      // Pass frame to the hue-model-based tracker:
      detection.setFrame(cv_ptr->image);
      std::vector<HueItem> detected = detection.detectAll(true);
      // Print detected items:
      // for (int i = 0; i < detected.size(); i++) {
      //  	printf("[%d] Detected: centroid = {%d, %d}, area = %lf, detector = {%s}\n", i, detected[i].centroid.x, detected[i].centroid.y, detected[i].area, detected[i].detector_id.c_str());
      // }
      image_converter.publish(cv_ptr);  // TODO: Optional!

      try {
        geometry_msgs::TransformStamped camera_link_tf = tf_buffer.lookupTransform(tf_prefix + "/camera_link", "map", ros::Time(0));
        tf2::Stamped<tf2::Transform> camera_link_tf2;
        tf2::fromMsg(camera_link_tf, camera_link_tf2);
        camera.R = link_to_cv * camera_link_tf2.getBasis();
        camera.T = link_to_cv * camera_link_tf2.getOrigin();
        sensed_pub.publish(fromHueItem(detected, camera));

      } catch (tf2::TransformException &e) {
        ROS_WARN("%s", e.what());
        continue;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
