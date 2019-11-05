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

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/Pose.h>

#include <drone_detector_tracking/moving_object_detector_reconfigureConfig.h>

namespace mbzirc
{
class MovingObjectDetector;
class MovingObjectParameters;

class MovingObjectDetectorHandler
{
  public:
   MovingObjectDetectorHandler(std::string node_name);
   virtual ~MovingObjectDetectorHandler();

  private:
   void loadParameters();

   void imageCb(const sensor_msgs::ImageConstPtr& rgb_img_msg);

   void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

   void poseCb(const geometry_msgs::PoseConstPtr& pose_msg);

   void publishDetectionMarkers();

   void reconfigure(drone_detector_tracking::moving_object_detector_reconfigureConfig& config, uint32_t level);

   MovingObjectDetector* _moving_object_detector = {nullptr};

   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;

   ros::Publisher _detection_markers_pub;

   image_transport::Publisher _tracking_pub;

   dynamic_reconfigure::Server<drone_detector_tracking::moving_object_detector_reconfigureConfig> _reconfig_server;
   dynamic_reconfigure::Server<drone_detector_tracking::moving_object_detector_reconfigureConfig>::CallbackType
       _reconfig_f;

   ros::Subscriber _rgb_img_sub;
   ros::Subscriber _camera_info_sub;
   ros::Subscriber _pose_sub;

   MovingObjectParameters* _movingObjectParams;

   std::string _node_name;
   std::string _rgb_img_topic;
   std::string _camera_info_topic;
   std::string _pose_topic;
};

}  // namespace mbzirc