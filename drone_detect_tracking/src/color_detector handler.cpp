/*!
 * @file drone_detector_handler.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 10/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <drone_detector_tracking/color_detector.h>
#include <drone_detector_tracking/color_detector_handler.h>
#include <drone_detector_tracking/reconfigure_parameters.h>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <mbzirc_comm_objs/ObjectDetectionList.h>

namespace mbzirc
{
ColorDetectorHandler::ColorDetectorHandler(std::string node_name) : _nh("~"), _it(_nh)
{
   _node_name = node_name;
   _params    = new ColorReconfigureParameters();
   ROS_INFO("[%s] Node handler initialization", _node_name.c_str());

   loadParameters();
   loadTopics();

   _camera_tf_listener = new tf2_ros::TransformListener(_tf_buffer);

   // listenBaseToCameraTf();
}

ColorDetectorHandler::~ColorDetectorHandler() {}

void ColorDetectorHandler::loadParameters()
{
   _nh.param<std::string>("rgb_img_topic", _rgb_img_topic, "/camera/image_raw");
   _nh.param<std::string>("camera_info_topic", _camera_info_topic, "/camera/camera_info");

   _nh.param<std::string>("pose_topic", _pose_topic, "/dji_control/pose");

   _nh.param<std::string>("base_link_name", _base_link_name, "base_link");
   _nh.param<std::string>("camera_link_name", _camera_link_name, "camera_link");

   _nh.param<float>("ball_diameter", _params->ball_diameter, 0.120);

   _reconfig_f = boost::bind(&ColorDetectorHandler::reconfigure, this, _1, _2);
   _reconfig_server.setCallback(_reconfig_f);

   ROS_INFO("[%s] Parameters loaded", _node_name.c_str());
}

void ColorDetectorHandler::loadTopics()
{
   _rgb_pub = _it.advertise("result/rgb", 1);
   // _dbg1_pub = _it.advertise("debug/dbg1", 1);
   // _dbg2_pub = _it.advertise("debug/dbg2", 1);
   // _dbg3_pub = _it.advertise("debug/dbg3", 1);
   // _dbg4_pub = _it.advertise("debug/dbg4", 1);

   _detection_list_pub = _nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1);

   _rgb_img_sub     = _nh.subscribe(_rgb_img_topic, 1, &ColorDetectorHandler::rgbImgCb, this);
   _camera_info_sub = _nh.subscribe(_camera_info_topic, 1, &ColorDetectorHandler::cameraInfoCb, this);
   _pose_sub        = _nh.subscribe(_pose_topic, 1, &ColorDetectorHandler::poseCb, this);
}

void ColorDetectorHandler::rgbImgCb(const sensor_msgs::ImageConstPtr& rgb_img_msg)
{
   cv_bridge::CvImagePtr rgb_img_ptr;
   try
   {
      rgb_img_ptr = cv_bridge::toCvCopy(rgb_img_msg, "rgb8");
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("[%s] cv_bridge exception: %s", _node_name.c_str(), e.what());
      return;
   }

   cv::Mat dbg1, dbg2, dbg3, dbg4;
   _color_detector->processFrame(rgb_img_ptr->image, dbg1, dbg2, dbg3, dbg4);

   publishImages(rgb_img_ptr->image, dbg1, dbg2, dbg3, dbg4, rgb_img_msg->header);
   publishDetections();
}

void ColorDetectorHandler::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
   CameraParameters camera_params;
   camera_params.intrinsics.fx = camera_info_msg->K[0];
   camera_params.intrinsics.fy = camera_info_msg->K[4];
   camera_params.intrinsics.cx = camera_info_msg->K[2];
   camera_params.intrinsics.cy = camera_info_msg->K[5];

   if (camera_info_msg->distortion_model == "plumb_bob")
   {
      camera_params.distortion.k1 = camera_info_msg->D[0];
      camera_params.distortion.k2 = camera_info_msg->D[1];
      camera_params.distortion.p1 = camera_info_msg->D[2];
      camera_params.distortion.p2 = camera_info_msg->D[3];
      camera_params.distortion.k3 = camera_info_msg->D[4];
   }

   _color_detector->setCameraCalibration(camera_params);
   _camera_info_sub.shutdown();
}

void ColorDetectorHandler::poseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
   Eigen::Isometry3d pose;

   tf::poseMsgToEigen(pose_msg->pose, pose);
   _color_detector->updateDronePose(pose);
   listenBaseToCameraTf();
}

void ColorDetectorHandler::listenBaseToCameraTf()
{
   geometry_msgs::TransformStamped transformStamped;
   try
   {
      Eigen::Isometry3d base_to_camera_tf;

      transformStamped = _tf_buffer.lookupTransform(_base_link_name, _camera_link_name, ros::Time(0), ros::Duration(2));

      tf::transformMsgToEigen(transformStamped.transform, base_to_camera_tf);

      if (!base_to_camera_tf.linear().isUnitary()) ROS_ERROR("Base to Camera link transform not unitary");

      _color_detector->setBaseToCameraTf(base_to_camera_tf);
   }
   catch (tf2::TransformException& ex)
   {
      ROS_WARN("%s", ex.what());
      ROS_WARN("Drone base to camera transform not set, considering no transformation.");

      _color_detector->setBaseToCameraTf(Eigen::Isometry3d::Identity());
      return;
   }
}

void ColorDetectorHandler::publishDetections()
{
   static int lastest_marker_id = 0;

   std::vector<Eigen::Vector3d> detections_positions          = _color_detector->getDetectionPositions();
   std::vector<Eigen::Vector3d> detections_relative_positions = _color_detector->getDetectionRelativePositions();

   mbzirc_comm_objs::ObjectDetectionList detection_list;
   for (size_t i = 0; i < detections_positions.size(); i++)
   {
      geometry_msgs::Point point_msg;
      point_msg.x = detections_positions[i].x();
      point_msg.y = detections_positions[i].y();
      point_msg.z = detections_positions[i].z();

      mbzirc_comm_objs::ObjectDetection object;
      object.type                    = "ball";
      object.pose.pose.position      = point_msg;
      object.pose.pose.orientation.w = 1;

      point_msg.x = detections_relative_positions[i].x();
      point_msg.y = detections_relative_positions[i].y();
      point_msg.z = detections_relative_positions[i].z();

      object.relative_position = point_msg;
      object.scale.x = object.scale.y = object.scale.z = _ball_diameter;
      object.color                                     = 0;

      detection_list.objects.push_back(object);
   }
   _detection_list_pub.publish(detection_list);
}

void ColorDetectorHandler::reconfigure(drone_detector_tracking::color_detector_reconfigureConfig& config, uint32_t)
{
   _params->h_upper_th = config.h_high;
   _params->h_lower_th = config.h_low;
   _params->s_upper_th = config.s_high;
   _params->s_lower_th = config.s_low;
   _params->v_upper_th = config.v_high;
   _params->v_lower_th = config.v_low;

   _params->blur_size     = config.blur_size;
   _params->opening_size  = config.opening_size;
   _params->dilation_size = config.dilation_size;

   _params->min_circularity = config.min_circularity;
   _params->max_circularity = config.max_circularity;

   if (_color_detector == nullptr)
   {
      _color_detector  = new ColorDetector(_params);
      _camera_info_sub = _nh.subscribe(_camera_info_topic, 1, &ColorDetectorHandler::cameraInfoCb, this);
      return;
   }

   _color_detector->setParams(_params);
}

void ColorDetectorHandler::publishImages(cv::Mat& img, cv::Mat& dbg1, cv::Mat& dbg2, cv::Mat& dbg3, cv::Mat& dbg4,
                                         std_msgs::Header header)
{
   // cv_bridge::CvImage cv_bridge = cv_bridge::CvImage(header, "rgb8", img);
   // sensor_msgs::Image debug_msg;
   // cv_bridge.toImageMsg(debug_msg);
   // _rgb_pub.publish(debug_msg);

   // cv_bridge::CvImage cv_dbg1_bridge = cv_bridge::CvImage(header, "rgb8", dbg1);
   // sensor_msgs::Image dbg1_msg;
   // cv_dbg1_bridge.toImageMsg(dbg1_msg);
   // _dbg1_pub.publish(dbg1_msg);

   cv_bridge::CvImage cv_dbg2_bridge = cv_bridge::CvImage(header, "rgb8", dbg2);
   sensor_msgs::Image dbg2_msg;
   cv_dbg2_bridge.toImageMsg(dbg2_msg);
   _rgb_pub.publish(dbg2_msg);

   // cv_bridge::CvImage cv_dbg3_bridge = cv_bridge::CvImage(header, "mono8", dbg3);
   // sensor_msgs::Image dbg3_msg;
   // cv_dbg3_bridge.toImageMsg(dbg3_msg);
   // _dbg3_pub.publish(dbg3_msg);

   // cv_bridge::CvImage cv_dbg4_bridge = cv_bridge::CvImage(header, "rgb8", dbg4);
   // sensor_msgs::Image dbg4_msg;
   // cv_dbg4_bridge.toImageMsg(dbg4_msg);
   // _dbg4_pub.publish(dbg4_msg);
}

}  // namespace mbzirc