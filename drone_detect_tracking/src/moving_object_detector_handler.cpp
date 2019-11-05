/*!
 * @file moving_object_detector_handler.cpp
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

#include <cv_bridge/cv_bridge.h>

#include <drone_detector_tracking/moving_object_detector.h>
#include <drone_detector_tracking/moving_object_detector_handler.h>
#include <drone_detector_tracking/moving_object_parameters.h>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace mbzirc
{
MovingObjectDetectorHandler::MovingObjectDetectorHandler(std::string node_name) : _nh("~"), _it(_nh)
{
   _node_name = node_name;

   ROS_INFO("[%s] Node handler initialization", _node_name.c_str());

   loadParameters();

   _movingObjectParams = new MovingObjectParameters();

   _reconfig_f = boost::bind(&MovingObjectDetectorHandler::reconfigure, this, _1, _2);
   _reconfig_server.setCallback(_reconfig_f);

   _rgb_img_sub     = _nh.subscribe(_rgb_img_topic, 1, &MovingObjectDetectorHandler::imageCb, this);
   _camera_info_sub = _nh.subscribe(_camera_info_topic, 1, &MovingObjectDetectorHandler::cameraInfoCb, this);
   _pose_sub        = _nh.subscribe(_pose_topic, 1, &MovingObjectDetectorHandler::poseCb, this);

   _tracking_pub = _it.advertise("tracking_result", 1);
}

MovingObjectDetectorHandler::~MovingObjectDetectorHandler() {}

void MovingObjectDetectorHandler::loadParameters()
{
   _nh.param<std::string>("rgb_img_topic", _rgb_img_topic, "/camera/color/image_rect_color");
   _nh.param<std::string>("camera_info_topic", _camera_info_topic, "/camera/color/camera_info");
   _nh.param<std::string>("uav_pose_topic", _pose_topic, "/dji_control/pose");

   ROS_INFO("[%s] Parameters loaded", _node_name.c_str());
}

void MovingObjectDetectorHandler::imageCb(const sensor_msgs::ImageConstPtr& rgb_img_msg)
{
   if (_moving_object_detector == nullptr) return;

   cv_bridge::CvImagePtr rgb_ptr;

   try
   {
      rgb_ptr = cv_bridge::toCvCopy(rgb_img_msg, "bgr8");
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("[%s] cv_bridge exception: %s", _node_name.c_str(), e.what());
      return;
   }

   _moving_object_detector->updateLK(rgb_ptr->image);

   cv_bridge::CvImage cv_bridge = cv_bridge::CvImage(rgb_img_msg->header, "bgr8", rgb_ptr->image);
   sensor_msgs::Image result_msg;
   cv_bridge.toImageMsg(result_msg);
   _tracking_pub.publish(result_msg);
}

void MovingObjectDetectorHandler::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
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

   _camera_info_sub.shutdown();
}

void MovingObjectDetectorHandler::poseCb(const geometry_msgs::PoseConstPtr& pose_msg)
{
   Eigen::Isometry3d pose;

   tf::poseMsgToEigen(*pose_msg, pose);
}

void MovingObjectDetectorHandler::reconfigure(drone_detector_tracking::moving_object_detector_reconfigureConfig& config,
                                              uint32_t)
{
   _movingObjectParams->velocity_tolerance = config.velocity_tolerance;

   _movingObjectParams->static_points_num  = config.static_points;
   _movingObjectParams->dynamic_points_num = config.dynamic_points;

   if (_moving_object_detector == nullptr)
   {
      _moving_object_detector = new MovingObjectDetector(_movingObjectParams);
      ROS_INFO("[%s] First reconfigure callback. Creating MovingObjectDetector", _node_name.c_str());
      return;
   }

   _moving_object_detector->setParams(_movingObjectParams);
}

}  // namespace mbzirc