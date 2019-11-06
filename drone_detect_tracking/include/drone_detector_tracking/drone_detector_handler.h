/*!
 * @file drone_detector_handler.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 10/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC

 * Copyright (c) 2019, FADA-CATEC
 */
#pragma once

#include <ros/ros.h>

#include <opencv2/core/mat.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CompressedImage.h>

#include <drone_detector_tracking/drone_detector_reconfigureConfig.h>

namespace mbzirc
{
class DroneDetector;
class ReconfigureParameters;

class DroneDetectorHandler
{
  public:
   DroneDetectorHandler(std::string node_name);
   virtual ~DroneDetectorHandler();

  private:
   void loadParameters();
   void loadTopics();

   void depthImgCb(const sensor_msgs::ImageConstPtr& depth_img_msg);
   void rgbImgCb(const sensor_msgs::CompressedImageConstPtr& rgb_img_msg);

   void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

   void poseCb(const geometry_msgs::PoseStampedConstPtr& pose_msg);

   void listenBaseToCameraTf();

   void publishDetectionMarkers();

   void reconfigure(drone_detector_tracking::drone_detector_reconfigureConfig& config, uint32_t level);

   DroneDetector* _drone_detector = {nullptr};

   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;

   ros::Publisher _detection_list_pub;
   ros::Publisher _detection_points_pub;
   ros::Publisher _detection_relative_points_pub;

   image_transport::Publisher _depth_pub;
   image_transport::Publisher _rgb_pub;

   dynamic_reconfigure::Server<drone_detector_tracking::drone_detector_reconfigureConfig> _reconfig_server;
   dynamic_reconfigure::Server<drone_detector_tracking::drone_detector_reconfigureConfig>::CallbackType _reconfig_f;

   ros::Subscriber _depth_img_sub;
   ros::Subscriber _rgb_img_sub;
   ros::Subscriber _camera_info_sub;
   ros::Subscriber _pose_sub;

   tf2_ros::Buffer _tf_buffer;
   tf2_ros::TransformListener* _camera_tf_listener;

   ReconfigureParameters* _reconfigureParams;

   std::string _node_name;
   std::string _depth_img_topic;
   std::string _rgb_img_topic;
   std::string _camera_info_topic;
   std::string _pose_topic;

   std::string _base_link_name;
   std::string _camera_link_name;

   bool _publish_debug_images;

   cv::Mat _rgb_img;
};

}  // namespace mbzirc