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

#include <drone_detector_tracking/drone_detector.h>
#include <drone_detector_tracking/drone_detector_handler.h>
#include <drone_detector_tracking/reconfigure_parameters.h>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

namespace mbzirc
{
DroneDetectorHandler::DroneDetectorHandler(std::string node_name) : _nh("~"), _it(_nh)
{
   _node_name = node_name;

   ROS_INFO("[%s] Node handler initialization", _node_name.c_str());

   loadParameters();
   loadTopics();

   _camera_tf_listener = new tf2_ros::TransformListener(_tf_buffer);

   listenBaseToCameraTf();
}

DroneDetectorHandler::~DroneDetectorHandler() {}

void DroneDetectorHandler::loadParameters()
{
   _nh.param<std::string>("depth_img_topic", _depth_img_topic, "/camera/aligned_depth_to_color/image_raw");
   _nh.param<std::string>("rgb_img_topic", _rgb_img_topic, "/camera/image_raw");
   _nh.param<std::string>("camera_info_topic", _camera_info_topic, "/camera_info");
   _nh.param<std::string>("uav_pose_topic", _pose_topic, "/m600/dji_control/pose");

   _nh.param<bool>("publish_debug_images", _publish_debug_images, "true");

   _nh.param<std::string>("base_link_name", _base_link_name, "base_link");
   _nh.param<std::string>("camera_link_name", _camera_link_name, "camera_link");

   _reconfigureParams = new ReconfigureParameters();

   _reconfig_f = boost::bind(&DroneDetectorHandler::reconfigure, this, _1, _2);
   _reconfig_server.setCallback(_reconfig_f);

   ROS_INFO("[%s] Parameters loaded", _node_name.c_str());
}

void DroneDetectorHandler::loadTopics()
{
   _detection_markers_pub         = _nh.advertise<visualization_msgs::MarkerArray>("detection_markers", 1);
   _detection_points_pub          = _nh.advertise<geometry_msgs::PointStamped>("detection_points", 1);
   _detection_relative_points_pub = _nh.advertise<geometry_msgs::PointStamped>("detection_relative_points", 1);
   _depth_pub                     = _it.advertise("result/depth", 1);
   _rgb_pub                       = _it.advertise("result/rgb", 1);

   _depth_img_sub   = _nh.subscribe(_depth_img_topic, 1, &DroneDetectorHandler::depthImgCb, this);
   _rgb_img_sub     = _nh.subscribe(_rgb_img_topic, 1, &DroneDetectorHandler::rgbImgCb, this);
   _camera_info_sub = _nh.subscribe(_camera_info_topic, 1, &DroneDetectorHandler::cameraInfoCb, this);
   _pose_sub        = _nh.subscribe(_pose_topic, 1, &DroneDetectorHandler::poseCb, this);
}

void DroneDetectorHandler::depthImgCb(const sensor_msgs::ImageConstPtr& depth_img_msg)
{
   if (_drone_detector == nullptr) return;
   if (_rgb_img.empty()) return;

   cv_bridge::CvImagePtr depth_ptr;

   try
   {
      depth_ptr = cv_bridge::toCvCopy(depth_img_msg, "32FC1");
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("[%s] cv_bridge exception: %s", _node_name.c_str(), e.what());
      return;
   }

   _drone_detector->processFrame(depth_ptr->image, _rgb_img);

   cv_bridge::CvImage cv_bridge_depth = cv_bridge::CvImage(depth_img_msg->header, "bgr8", depth_ptr->image);
   sensor_msgs::Image depth_result_msg;
   cv_bridge_depth.toImageMsg(depth_result_msg);
   _depth_pub.publish(depth_result_msg);

   cv_bridge::CvImage cv_bridge_rgb = cv_bridge::CvImage(depth_img_msg->header, "bgr8", _rgb_img);
   sensor_msgs::Image rgb_result_msg;
   cv_bridge_rgb.toImageMsg(rgb_result_msg);
   _rgb_pub.publish(rgb_result_msg);

   if (_publish_debug_images)
   {
      std::vector<cv::Mat> debug_images = _drone_detector->getThresholdsDebugImages();
      static std::vector<image_transport::Publisher> debug_img_pubs;

      static bool initialized_debug_topics;
      if (!initialized_debug_topics)
      {
         debug_img_pubs.resize(debug_images.size());
         for (size_t i = 0; i < debug_img_pubs.size(); i++)
         {
            debug_img_pubs[i] = _it.advertise("debug_" + std::to_string(i), 1);
         }

         initialized_debug_topics = true;
      }

      for (size_t i = 0; i < debug_images.size(); i++)
      {
         if (debug_images.size() > debug_img_pubs.size())
         {
            initialized_debug_topics = false;
            break;
         }

         cv_bridge::CvImage cv_db_bridge = cv_bridge::CvImage(depth_img_msg->header, "rgb8", debug_images[i]);
         sensor_msgs::Image debug_msg;
         cv_db_bridge.toImageMsg(debug_msg);
         debug_img_pubs[i].publish(debug_msg);
      }
   }

   publishDetectionMarkers();
}

void DroneDetectorHandler::rgbImgCb(const sensor_msgs::CompressedImageConstPtr& rgb_img_msg)
{
   try
   {
      _rgb_img = cv::imdecode(cv::Mat(rgb_img_msg->data), 1);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("[%s] cv_bridge exception: %s", _node_name.c_str(), e.what());
      return;
   }
}

void DroneDetectorHandler::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
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

   _drone_detector->setCameraCalibration(camera_params);
   _camera_info_sub.shutdown();
}

void DroneDetectorHandler::poseCb(const geometry_msgs::PoseConstPtr& pose_msg)
{
   Eigen::Isometry3d pose;

   tf::poseMsgToEigen(*pose_msg, pose);

   _drone_detector->updateDronePose(pose);
}

void DroneDetectorHandler::listenBaseToCameraTf()
{
   geometry_msgs::TransformStamped transformStamped;
   try
   {
      Eigen::Isometry3d base_to_camera_tf;

      transformStamped = _tf_buffer.lookupTransform(_base_link_name, _camera_link_name, ros::Time(0), ros::Duration(1));

      tf::transformMsgToEigen(transformStamped.transform, base_to_camera_tf);

      if (!base_to_camera_tf.linear().isUnitary()) ROS_ERROR("Base to Camera link transform not unitary");

      _drone_detector->setBaseToCameraTf(base_to_camera_tf);
   }
   catch (tf2::TransformException& ex)
   {
      ROS_WARN("%s", ex.what());
      ROS_WARN("Drone base to camera transform not set, considering no transformation.");

      _drone_detector->setBaseToCameraTf(Eigen::Isometry3d::Identity());
      return;
   }
}

void DroneDetectorHandler::publishDetectionMarkers()
{
   static int lastest_marker_id = 0;

   std::vector<Eigen::Vector3d> detections_positions          = _drone_detector->getDetectionPositions();
   std::vector<Eigen::Vector3d> detections_relative_positions = _drone_detector->getDetectionRelativePositions();

   visualization_msgs::MarkerArray markers_array_msg;
   for (auto position : detections_positions)
   {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp    = ros::Time::now();
      marker.pose.position.x = position.x();
      marker.pose.position.y = position.y();
      marker.pose.position.z = position.z();
      marker.scale.x         = 0.1;
      marker.scale.y         = 0.1;
      marker.scale.z         = 0.1;
      marker.color.a         = 1.0;  // Don't forget to set the alpha!
      marker.color.g         = 1.0;
      marker.ns              = _node_name;
      marker.id              = lastest_marker_id;
      marker.type            = visualization_msgs::Marker::SPHERE;
      marker.action          = visualization_msgs::Marker::ADD;
      lastest_marker_id++;
      markers_array_msg.markers.push_back(marker);

      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = "map";
      point_msg.header.stamp    = ros::Time::now();
      point_msg.point.x         = position.x();
      point_msg.point.y         = position.y();
      point_msg.point.z         = position.z();

      _detection_points_pub.publish(point_msg);
   }

   for (auto position : detections_relative_positions)
   {
      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = "map";
      point_msg.header.stamp    = ros::Time::now();
      point_msg.point.x         = position.x();
      point_msg.point.y         = position.y();
      point_msg.point.z         = position.z();

      _detection_relative_points_pub.publish(point_msg);
   }

   if (markers_array_msg.markers.size() > 0)
   {
      _detection_markers_pub.publish(markers_array_msg);
   }
}

void DroneDetectorHandler::reconfigure(drone_detector_tracking::drone_detector_reconfigureConfig& config, uint32_t)
{
   _reconfigureParams->erosion_size = config.erosion_size;

   _reconfigureParams->unit_depth = config.unit_depth;
   _reconfigureParams->min_depth  = config.min_depth;
   _reconfigureParams->max_depth  = config.max_depth;
   _reconfigureParams->step_depth = config.step_depth;

   _reconfigureParams->min_area = config.min_area;
   _reconfigureParams->max_area = config.max_area;

   _reconfigureParams->min_circularity = config.min_circularity;
   _reconfigureParams->max_circularity = config.max_circularity;

   _reconfigureParams->min_convexity = config.min_convexity;
   _reconfigureParams->max_convexity = config.max_convexity;

   _reconfigureParams->max_dist_local_detections = config.max_dist_local_detections;
   _reconfigureParams->max_dist_detections       = config.max_dist_detections;
   _reconfigureParams->min_group_size            = config.min_group_size;

   _reconfigureParams->sigma_xy = config.sigma_xy;
   _reconfigureParams->sigma_z  = config.sigma_z;

   if (_drone_detector == nullptr)
   {
      _drone_detector = new DroneDetector(_reconfigureParams, _publish_debug_images);
      ROS_INFO("[%s] First reconfigure callback. Creating DroneDetector", _node_name.c_str());
      return;
   }

   _drone_detector->setParams(_reconfigureParams);
}

}  // namespace mbzirc