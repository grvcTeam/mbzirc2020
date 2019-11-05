/*!
 * @file color_detector_handler.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 8/10/2019
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
#include <sensor_msgs/Image.h>

#include <drone_detector_tracking/color_detector_reconfigureConfig.h>
#include <drone_detector_tracking/reconfigure_parameters.h>

namespace mbzirc
{
class ColorDetector;
class ReconfigureParameters;

class ColorDetectorHandler
{
  public:
   ColorDetectorHandler(std::string node_name);
   virtual ~ColorDetectorHandler();

  private:
   void loadParameters();
   void loadTopics();

   void rgbImgCb(const sensor_msgs::ImageConstPtr& rgb_img_msg);

   void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

   void poseCb(const geometry_msgs::PoseStampedConstPtr& rgb_img_msg);

   void listenBaseToCameraTf();

   void publishDetections();

   void reconfigure(drone_detector_tracking::color_detector_reconfigureConfig& config, uint32_t level);
   void publishImages(cv::Mat& img, cv::Mat& dbg1, cv::Mat& dbg2, cv::Mat& dbg3, cv::Mat& dbg4,
                      std_msgs::Header header);

   ColorDetector* _color_detector = {nullptr};

   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;

   ros::Publisher _detection_list_pub;

   image_transport::Publisher _rgb_pub;
   image_transport::Publisher _dbg1_pub;
   image_transport::Publisher _dbg2_pub;
   image_transport::Publisher _dbg3_pub;
   image_transport::Publisher _dbg4_pub;

   dynamic_reconfigure::Server<drone_detector_tracking::color_detector_reconfigureConfig> _reconfig_server;
   dynamic_reconfigure::Server<drone_detector_tracking::color_detector_reconfigureConfig>::CallbackType _reconfig_f;

   ros::Subscriber _rgb_img_sub;
   ros::Subscriber _camera_info_sub;
   ros::Subscriber _pose_sub;

   tf2_ros::Buffer _tf_buffer;
   tf2_ros::TransformListener* _camera_tf_listener;

   ColorReconfigureParameters* _params;

   float _ball_diameter;

   std::string _node_name;
   std::string _rgb_img_topic;
   std::string _camera_info_topic;

   std::string _pose_topic;

   std::string _base_link_name;
   std::string _camera_link_name;
};

}  // namespace mbzirc