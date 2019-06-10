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

bool geoLocate(const cv::Point& _image_point, geometry_msgs::Point *_world_point, const tf2::Matrix3x3& _K, const tf2::Matrix3x3& _R, const tf2::Vector3& _T, double _estimated_z) {

	tf2::Vector3 ray;
	double aux = (_image_point.y - _K.getRow(1)[2]) / _K.getRow(1)[1];
	ray[0] = (_image_point.x - _K.getRow(0)[2] - _K.getRow(0)[1] * aux) / _K.getRow(0)[0];
  ray[1] = aux;
  ray[2] = 1.0;
  // TODO: We end using only _R.transpose()...
  const tf2::Matrix3x3 Rt = _R.transpose();
  tf2::Vector3 ray_world = Rt * ray;
  tf2::Vector3 T_world = Rt * _T;
  if (ray_world[2] == 0) {
    ROS_WARN("geoLocate: ray_world[2] == 0");
    return false;
  }
	// The line equation is X = lambda * ray_world - T_world
	// lambda can be set because the z is known (estimated)
	double lambda = (_estimated_z + T_world[2]) / ray_world[2];
  _world_point->x = lambda * ray_world[0] - T_world[0];
  _world_point->y = lambda * ray_world[1] - T_world[1];
  _world_point->z = lambda * ray_world[2] - T_world[2];
  return true;
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
  const tf2::Matrix3x3 camera_K(fx,gamma,u0, 0,fy,v0, 0,0,1);

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
        tf2::Matrix3x3 camera_R = link_to_cv * camera_link_tf2.getBasis();
        tf2::Vector3   camera_T = link_to_cv * camera_link_tf2.getOrigin();
        mbzirc_comm_objs::ObjectDetectionList object_list;
        for (auto item: detected) {
          geometry_msgs::Point position;
          if (geoLocate(item.centroid, &position, camera_K, camera_R, camera_T, 0.0)) {  // TODO: z_estimation based on brick height
            mbzirc_comm_objs::ObjectDetection object;
            object.header.frame_id = "map";
            object.header.stamp = ros::Time::now();
            object.type = "brick";
            object.pose.pose.position = position;  // TODO: Covariance and orientation?
            object.pose.covariance[0] = 0.01;
            object.pose.covariance[7] = 0.01;
            object.pose.covariance[14] = 0.01;
            object.scale.x = 1;  // TODO: Something based in item.area
            object.scale.y = 1;
            object.scale.z = 1;
            object.properties = "{color: \'" + item.detector_id + "\'}";
            object_list.objects.push_back(object);
            sensed_pub.publish(object_list);
            // std::cout << object << '\n';
          }
        }
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
