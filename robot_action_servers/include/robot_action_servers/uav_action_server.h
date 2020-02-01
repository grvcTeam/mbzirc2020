//----------------------------------------------------------------------------------------------------------------------
// GRVC MBZIRC
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2019 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef UAV_ACTION_SERVER
#define UAV_ACTION_SERVER

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <actionlib/server/simple_action_server.h>
#include <mbzirc_comm_objs/TakeOffAction.h>
#include <mbzirc_comm_objs/GoToAction.h>
#include <mbzirc_comm_objs/PickAction.h>
#include <mbzirc_comm_objs/PlaceAction.h>
#include <mbzirc_comm_objs/LandAction.h>
#include <mbzirc_comm_objs/MoveInCirclesAction.h>
#include <mbzirc_comm_objs/ExtinguishFacadeFireAction.h>
#include <mbzirc_comm_objs/ExtinguishGroundFireAction.h>
#include <mbzirc_comm_objs/RobotDataFeed.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/GripperAttached.h>
#include <mbzirc_comm_objs/DetectTypes.h>
#include <mbzirc_comm_objs/Magnetize.h>
#include <mbzirc_comm_objs/WallList.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/posePID.h>
#include <handy_tools/pid_controller.h>
#include <handy_tools/circular_buffer.h>

#define Z_GIVE_UP_CATCHING 17.5  // TODO: From config file?
#define Z_RETRY_CATCH 1.0
#define CANDIDATE_TIMEOUT 1.5  // [s]
#define CATCHING_LOOP_RATE 20  // [Hz]
#define GROUND_EXTINGUISH_LOOP_RATE 10  // [Hz]
#define FACADE_EXTINGUISH_LOOP_RATE 20  // [Hz]
#define AVG_XY_ERROR_WINDOW_SIZE 13
#define AVG_Z_ERROR_WINDOW_SIZE 13
#define MAX_DELTA_Z 0.6  // [m] --> [m/s]
#define MAX_AVG_XY_ERROR 0.15  // [m]
#define MAX_AVG_Z_ERROR 0.2  // [m]
#define RELEASE_Z_ERROR_THRESHOLD 0.1 // [m]
#define RELEASE_XY_ERROR_THRESHOLD 0.1 // [m]

// TODO: From utils?
inline double normalizeAngle(double angle) {
    while (angle < -M_PI) angle += 2*M_PI;
    while (angle >  M_PI) angle -= 2*M_PI;
    return angle;
}

class UalActionServer {
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs
  actionlib::SimpleActionServer<mbzirc_comm_objs::TakeOffAction> take_off_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::PickAction> pick_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::PlaceAction> place_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::LandAction> land_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::MoveInCirclesAction> move_in_circles_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::ExtinguishFacadeFireAction> extinguish_facade_fire_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::ExtinguishGroundFireAction> extinguish_ground_fire_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tf_listener_;
  std::string tf_prefix_;

  grvc::ual::UAL *ual_;
  ros::Timer data_feed_timer_;
  ros::Publisher data_feed_pub_;
  ros::Publisher marker_pub_;
  mbzirc_comm_objs::ObjectDetection matched_candidate_;
  bool gripper_attached_ = false;
  sensor_msgs::Range sf11_range_;
  mbzirc_comm_objs::WallList wall_list_;

public:

  UalActionServer();
  ~UalActionServer();

  // Common
  void publishDataFeed(const ros::TimerEvent&);
  void takeOffCallback(const mbzirc_comm_objs::TakeOffGoalConstPtr &_goal);
  void landCallback(const mbzirc_comm_objs::LandGoalConstPtr &_goal);
  void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal);
  void trackedBrickCallback(const mbzirc_comm_objs::ObjectDetectionConstPtr& msg);
  void trackedFireCallback(const mbzirc_comm_objs::ObjectDetectionConstPtr& msg);
  void sensedObjectCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg);
  void attachedCallback(const mbzirc_comm_objs::GripperAttachedConstPtr& msg);
  void sf11RangeCallback(const sensor_msgs::RangeConstPtr& msg);
  
  // Challenge 2
  void pickCallback(const mbzirc_comm_objs::PickGoalConstPtr &_goal);
  void placeCallback(const mbzirc_comm_objs::PlaceGoalConstPtr &_goal);

  // Challenge 3
  void wallListCallback(const mbzirc_comm_objs::WallListConstPtr& msg);
  void moveInCirclesCallback(const mbzirc_comm_objs::MoveInCirclesGoalConstPtr &_goal);
  void extinguishFacadeFireCallback(const mbzirc_comm_objs::ExtinguishFacadeFireGoalConstPtr &_goal);
  void extinguishGroundFireCallback(const mbzirc_comm_objs::ExtinguishGroundFireGoalConstPtr &_goal);

};

#endif // UAV_ACTION_SERVER