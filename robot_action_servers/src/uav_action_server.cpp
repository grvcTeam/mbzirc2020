//----------------------------------------------------------------------------------------------------------------------
// GRVC Aeroarms
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
#include <mbzirc_comm_objs/Magnetize.h>
#include <mbzirc_comm_objs/WallList.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/ual_backend_dummy.h>
#include <ual_backend_mavros/ual_backend_mavros.h>
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/posePID.h>
#include <handy_tools/pid_controller.h>
#include <handy_tools/circular_buffer.h>
#include <fire_extinguisher/fire_data.h>
#include <scan_passage_detection/wall_utils.h>

#define EXTINGUISH_LOOP_RATE 20  // [Hz]

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

  UalActionServer():
    take_off_server_(nh_, "take_off_action", boost::bind(&UalActionServer::takeOffCallback, this, _1), false),
    go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
    pick_server_(nh_, "pick_action", boost::bind(&UalActionServer::pickCallback, this, _1), false),
    place_server_(nh_, "place_action", boost::bind(&UalActionServer::placeCallback, this, _1), false),
    land_server_(nh_, "land_action", boost::bind(&UalActionServer::landCallback, this, _1), false),
    move_in_circles_server_(nh_, "move_in_circles_action", boost::bind(&UalActionServer::moveInCirclesCallback, this, _1), false),
    extinguish_facade_fire_server_(nh_, "extinguish_facade_fire_action", boost::bind(&UalActionServer::extinguishFacadeFireCallback, this, _1), false),
    extinguish_ground_fire_server_(nh_, "extinguish_ground_fire_action", boost::bind(&UalActionServer::extinguishGroundFireCallback, this, _1), false) {

    std::string ual_backend;
    ros::param::param<std::string>("~ual_backend", ual_backend, "mavros");
    ros::param::param<std::string>("~tf_prefix", tf_prefix_, "default_prefix");

    grvc::ual::Backend *backend = nullptr;
    if (ual_backend == "dummy") {
      backend = new grvc::ual::BackendDummy();
    } else if (ual_backend == "mavros") {
      backend = new grvc::ual::BackendMavros();
    } else if (ual_backend == "gazebo_light") {
      backend = new grvc::ual::BackendGazeboLight();
    } else {
      throw std::runtime_error("Unexpected UAL backend");
    }

    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    ual_ = new grvc::ual::UAL(backend);  // TODO: Wait until isReady()?
    data_feed_timer_ = nh_.createTimer(ros::Duration(0.1), &UalActionServer::publishDataFeed, this);  // TODO: frequency?
    data_feed_pub_ = nh_.advertise<mbzirc_comm_objs::RobotDataFeed>("data_feed", 1);  // TODO: namespacing?
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    // TODO: start servers only when needed?
    // TODO: make servers separated classes?
    take_off_server_.start();
    go_to_server_.start();
    pick_server_.start();
    place_server_.start();
    land_server_.start();
    move_in_circles_server_.start();
    extinguish_facade_fire_server_.start();
    extinguish_ground_fire_server_.start();
  }

  ~UalActionServer() {
    delete ual_;
  }

  void publishDataFeed(const ros::TimerEvent&) {
    mbzirc_comm_objs::RobotDataFeed data_feed;
    data_feed.pose = ual_->pose();
    data_feed_pub_.publish(data_feed);
  }

  // TODO: Who should know about flight_level? robot_action_servers, uav_agent, central_agent...
  void takeOffCallback(const mbzirc_comm_objs::TakeOffGoalConstPtr &_goal) {
    // ROS_INFO("Take off!");
    // mbzirc_comm_objs::TakeOffFeedback feedback;
    mbzirc_comm_objs::TakeOffResult result;

    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");
      sleep(1);
    }
    // TODO: Fill result
    switch (ual_->state().state) {
      case uav_abstraction_layer::State::LANDED_DISARMED:
        ROS_WARN("UAL is disarmed!");
        take_off_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDED_ARMED: {
        double current_z = ual_->pose().pose.position.z;
        ual_->takeOff(_goal->height - current_z, true);  // TODO: timeout? preempt?
        take_off_server_.setSucceeded(result);
        break;
      }
      case uav_abstraction_layer::State::TAKING_OFF:
        ROS_WARN("UAL is taking off!");
        // TODO: Wait until FLYING_AUTO?
        take_off_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::FLYING_AUTO:
        // Already flying!
        take_off_server_.setSucceeded(result);
        break;
      case uav_abstraction_layer::State::FLYING_MANUAL:
        ROS_WARN("UAL is flying manual!");
        take_off_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDING:
        ROS_WARN("UAL is landing!");
        // TODO: Wait until LANDED_ARMED and then take off?
        take_off_server_.setAborted(result);
        break;
      default:
        ROS_ERROR("Unexpected UAL state!");
        take_off_server_.setAborted(result);
    }
  }

  void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal) {
    // ROS_INFO("Go to!");
    // mbzirc_comm_objs::GoToFeedback feedback;
    mbzirc_comm_objs::GoToResult result;

    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");
      sleep(1);
    }
    // TODO: Fill result
    switch (ual_->state().state) {
      case uav_abstraction_layer::State::LANDED_DISARMED:
        ROS_WARN("UAL is disarmed!");
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDED_ARMED:
        // We could take_off, then go_to, but may be confusing
        ROS_WARN("UAL is landed!");
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::TAKING_OFF:
        ROS_WARN("UAL is taking off!");
        // TODO: Wait until FLYING_AUTO?
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::FLYING_AUTO:
        ual_->goToWaypoint(_goal->waypoint, true);  // TODO: timeout?
        go_to_server_.setSucceeded(result);
        break;
      case uav_abstraction_layer::State::FLYING_MANUAL:
        ROS_WARN("UAL is flying manual!");
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDING:
        ROS_WARN("UAL is landing!");
        // TODO: Wait until LANDED_ARMED and then take off?
        go_to_server_.setAborted(result);
        break;
      default:
        ROS_ERROR("Unexpected UAL state!");
        go_to_server_.setAborted(result);
    }
  }

  void trackedObjectCallback(const mbzirc_comm_objs::ObjectDetectionConstPtr& msg) {
    if (msg->type != mbzirc_comm_objs::ObjectDetection::TYPE_BRICK_TRACK) {
      ROS_WARN("Expected TYPE_BRICK_TRACK!");
      return;
    }
    matched_candidate_ = *msg;  // TODO: Check also color is correct?
  }

  void attachedCallback(const mbzirc_comm_objs::GripperAttachedConstPtr& msg) {
    gripper_attached_ = msg->attached;
  }

  void sf11RangeCallback(const sensor_msgs::RangeConstPtr& msg) {
    sf11_range_ = *msg;
  }

  void wallListCallback(const mbzirc_comm_objs::WallListConstPtr& msg) {
    wall_list_ = *msg;
  }

  void pickCallback(const mbzirc_comm_objs::PickGoalConstPtr &_goal) {
    // ROS_INFO("Pick!");
    // mbzirc_comm_objs::PickFeedback feedback;
    mbzirc_comm_objs::PickResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");
      pick_server_.setAborted(result);
      return;
    }

    geometry_msgs::TransformStamped camera_to_gripper;  // Constant!
    try {
      camera_to_gripper = tf_buffer_.lookupTransform(tf_prefix_ + "/gripper_link", tf_prefix_ + "/camera_color_optical_frame", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    ros::NodeHandle nh;
    ros::Subscriber sensed_sub_ = nh.subscribe("tracked_object", 1, &UalActionServer::trackedObjectCallback, this);
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
    ros::Subscriber attached_sub_ = nh.subscribe<mbzirc_comm_objs::GripperAttached>("actuators_system/gripper_attached", 1, &UalActionServer::attachedCallback, this);
    ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");  // TODO: New naming!
    ros::ServiceClient close_gripper_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/close_gripper");
    ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?

    // bool has_tracked;
    // bool has_sf11;
    // bool has_gripper;
    // for (int i = 0; i < 100; i++) {  // 100*0.1 = 10 seconds timeout
    //   has_tracked = (matched_candidate_.header.seq != 0);
    //   has_sf11 = (sf11_range_.header.seq != 0);
    //   // has_gripper = (gripper_attached_.header.seq != 0);
    //   has_gripper = true;  // TODO!
    //   if (has_tracked && has_sf11 && has_gripper) {
    //     break;
    //   }
    //   ros::Duration(0.1).sleep();
    // }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Tune!
    #define Z_GIVE_UP_CATCHING 17.5  // TODO: From config file?
    #define Z_RETRY_CATCH 1.0
    #define CANDIDATE_TIMEOUT 1.5  // [s]
    #define CATCHING_LOOP_RATE 10  // [Hz]
    #define AVG_XY_ERROR_WINDOW_SIZE 13
    #define MAX_DELTA_Z 0.25  // [m] --> [m/s]
    #define MAX_AVG_XY_ERROR 0.2  // [m]

    grvc::ual::PIDParams pid_x;
    pid_x.kp = 0.82;
    pid_x.ki = 0.0;
    pid_x.kd = 0.0;
    pid_x.min_sat = -2.0;
    pid_x.max_sat = 2.0;
    pid_x.min_wind = -2.0;
    pid_x.max_wind = 2.0;

    grvc::ual::PIDParams pid_y;
    pid_y.kp = 0.82;
    pid_y.ki = 0.0;
    pid_y.kd = 0.0;
    pid_y.min_sat = -2.0;
    pid_y.max_sat = 2.0;
    pid_y.min_wind = -2.0;
    pid_y.max_wind = 2.0;

    grvc::ual::PIDParams pid_z;
    pid_z.kp = 0.5;
    pid_z.ki = 0.0;
    pid_z.kd = 0.0;
    pid_z.min_sat = -2.0;
    pid_z.max_sat = 2.0;
    pid_z.min_wind = -2.0;
    pid_z.max_wind = 2.0;

    grvc::ual::PIDParams pid_yaw;
    pid_yaw.kp = 0.4;
    pid_yaw.ki = 0.02;
    pid_yaw.kd = 0.0;
    pid_yaw.min_sat = -2.0;
    pid_yaw.max_sat = 2.0;
    pid_yaw.min_wind = -2.0;
    pid_yaw.max_wind = 2.0;
    pid_yaw.is_angular = true;

    grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
    pose_pid.enableRosInterface("pick_control");

    // TODO: Magnetize catching device
    mbzirc_comm_objs::Magnetize magnetize_srv;
    magnetize_srv.request.magnetize = true;
    if (!magnetize_client.call(magnetize_srv)) {
      ROS_ERROR("Failed to call [magnetize] service");
    }

    grvc::utils::CircularBuffer history_xy_errors;
    history_xy_errors.set_size(AVG_XY_ERROR_WINDOW_SIZE);
    history_xy_errors.fill_with(MAX_AVG_XY_ERROR);
    // unsigned tries_counter = 0;  // TODO: as feedback?
    ros::Duration timeout(CANDIDATE_TIMEOUT);
    ros::Rate loop_rate(CATCHING_LOOP_RATE);
    while (ros::ok()) {
      ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
      // ROS_INFO("since_last_candidate = %lf, timeout = %lf", since_last_candidate.toSec(), timeout.toSec());

      if (pick_server_.isPreemptRequested()) {
        ual_->setPose(ual_->pose());
        pick_server_.setPreempted();
        return;
      }

      // bool so_far = (sf11_range_.range > 1.5);  // TODO: threshold!
      const float height_threshold = 0.7;
      geometry_msgs::PoseStamped error_pose;
      error_pose.header.stamp = ros::Time::now();
      error_pose.header.frame_id = tf_prefix_ + "/gripper_link";
      // geometry_msgs::Quaternion yaw_lock;
      if (!matched_candidate_.is_cropped && (sf11_range_.range > height_threshold)) {  // TODO: Threshold
        // TODO!
        error_pose.pose.position.x = -matched_candidate_.point_of_interest.y;
        error_pose.pose.position.y = -matched_candidate_.point_of_interest.x;
        error_pose.pose.position.z = -matched_candidate_.point_of_interest.z;
        // reference_pose.pose.position.x = -matched_candidate_.pose.pose.position.y;
        // reference_pose.pose.position.y = -matched_candidate_.pose.pose.position.x;
        // reference_pose.pose.position.z = -matched_candidate_.pose.pose.position.z;
        error_pose.pose.orientation = matched_candidate_.pose.pose.orientation;
        error_pose.pose.orientation.z = -error_pose.pose.orientation.z;  // Change sign!
        // yaw_lock = reference_pose.pose.orientation;
      } else {
        tf2::doTransform(matched_candidate_.pose.pose, error_pose.pose, camera_to_gripper);
        error_pose.header.frame_id = tf_prefix_ + "/gripper_link";
        error_pose.pose.orientation.x = 0;
        error_pose.pose.orientation.y = 0;
        error_pose.pose.orientation.z = 0;
        error_pose.pose.orientation.w = 1;  // TODO!
        // reference_pose.pose.orientation = yaw_lock;
        float dz = fabs(height_threshold - sf11_range_.range);
        error_pose.pose.position.x = (1.0-dz) * error_pose.pose.position.x - dz * matched_candidate_.point_of_interest.y;
        error_pose.pose.position.y = (1.0-dz) * error_pose.pose.position.y - dz * matched_candidate_.point_of_interest.x;
        error_pose.pose.position.z = (1.0-dz) * error_pose.pose.position.z - dz * matched_candidate_.point_of_interest.z;
      }

      geometry_msgs::Point error_position = error_pose.pose.position;
      // geometry_msgs::Point target_position = target_pose.position;
      // tf2_geometry_msgs::do_transform(matched_candidate_.pose.pose.position, target_position, camera_to_gripper);

      if (since_last_candidate < timeout) {
        // x-y-control: in candidateCallback
        // z-control: descend
        // ROS_ERROR("current: [%lf, %lf, %lf]", current_position.x, current_position.y, current_position.z);
        double xy_error = sqrt(error_position.x*error_position.x + error_position.y*error_position.y);  // TODO: sqrt?
        history_xy_errors.push(xy_error);
        double min_xy_error, avg_xy_error, max_xy_error;
        history_xy_errors.get_stats(min_xy_error, avg_xy_error, max_xy_error);

        if (avg_xy_error > MAX_AVG_XY_ERROR) {
          avg_xy_error = MAX_AVG_XY_ERROR;
        }
        error_pose.pose.position.z = -MAX_DELTA_Z * (1.0 - (avg_xy_error / MAX_AVG_XY_ERROR));
        // ROS_INFO("xy_error = %lf, avg_xy_error = %lf, target_position.z = %lf", xy_error, avg_xy_error, target_position.z);

      } else {  // No fresh candidates (timeout)
        ROS_WARN("Candidates timeout!");

        // TODO: Push MAX_AVG_XY_ERROR into history_xy_errors
        error_pose.pose.position.x = 0.0;
        error_pose.pose.position.y = 0.0;
        error_pose.pose.position.z = MAX_DELTA_Z;
        error_pose.pose.orientation.x = 0.0;
        error_pose.pose.orientation.y = 0.0;
        error_pose.pose.orientation.z = 0.0;
        error_pose.pose.orientation.w = 1.0;

        // // Go up in the same position.
        // grvc::ual::Waypoint up_waypoint = ual_.pose();
        // up_waypoint.pose.position.z = Z_RETRY_CATCH;
        // ual_.goToWaypoint(up_waypoint);  // Blocking!

        // // Move to target position.
        // grvc::ual::Waypoint approaching_waypoint = ual_.pose();
        // approaching_waypoint.pose.position.x = target_.global_position.x;
        // approaching_waypoint.pose.position.y = target_.global_position.y;
        // approaching_waypoint.pose.position.z = Z_RETRY_CATCH;
        // approaching_waypoint.header.frame_id = "map";
        // ual_.goToWaypoint(approaching_waypoint);  // Blocking!

        // tries_counter++;
        // if (tries_counter > max_tries_counter_) {
        //     // Go to initial catch position.
        //     grvc::ual::Waypoint initial_catch = ual_.pose();
        //     initial_catch.pose.position.x = target_.global_position.x;
        //     initial_catch.pose.position.y = target_.global_position.y;
        //     initial_catch.pose.position.z = flying_level_;
        //     initial_catch.header.frame_id = "map";                    
        //     ual_.goToWaypoint(initial_catch);

        //     // Set target to failed
        //     mbzirc_scheduler::SetTargetStatus target_status_call;
        //     target_status_call.request.target_id = target_.target_id;
        //     target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::FAILED;
        //     if (!target_status_client_.call(target_status_call)) {
        //             ROS_ERROR("Error setting target status to FAILED in UAV_%d", uav_id_);
        //     }                                           

        //     // Switch to HOVER state.
        //     hover_position_waypoint_ = ual_.pose();
        //     state_.state = UavState::HOVER;
        //     // Break loop.
        //     return;
        // }
      }
      // Send target_position  // TODO: find equivalent!
      geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);
      // ROS_ERROR("vel = [%lf, %lf, %lf]", velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);

      ual_->setVelocity(velocity);
      // ROS_INFO("Candidate relative position = [%lf, %lf, %lf]", matched_candidate_.relative_position.x, matched_candidate_.relative_position.y, matched_candidate_.relative_position.z);
      // ROS_INFO("target_position = [%lf, %lf, %lf] target angle = x", target_position.x, target_position.y, target_position.z);
      // ROS_INFO("velocity = [%lf, %lf, %lf]", velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);

      // TODO: Look for equivalent!
      // if (catching_device_->switchIsPressed()) {
      if (gripper_attached_) {
        std_srvs::Trigger dummy;
        close_gripper_client.call(dummy);
        // sleep(0.1);  // TODO: sleep?
        auto up_pose = ual_->pose();
        up_pose.pose.position.z += 2.0;  // TODO: Up?
        ual_->setPose(up_pose);
        pick_server_.setSucceeded(result);
        return;
        // break;
      }

      // If we're too high, give up TODO: use _goal.z?
      if (ual_->pose().pose.position.z > Z_GIVE_UP_CATCHING) {

        // TODO: Find equivalent            
        // mbzirc_scheduler::SetTargetStatus target_status_call;
        // target_status_call.request.target_id = target_.target_id;
        // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::LOST;
        // if (!target_status_client_.call(target_status_call)) {
        //   ROS_ERROR("Error setting target status to LOST in UAV_%d", uav_id_);
        // }
        // hover_position_waypoint_ = ual_.pose();
        pick_server_.setAborted(result);
        break;
      }

      // TODO: Review this frequency!
      loop_rate.sleep();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ual_->takeOff(_goal->waypoint.pose.position.z - ual_->pose().pose.position.z, true);  // TODO: timeout? preempt?
    // ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt?
    // ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt?
    sensed_sub_.shutdown();
  }

  void placeCallback(const mbzirc_comm_objs::PlaceGoalConstPtr &_goal) {
    // ROS_INFO("Place!");
    // mbzirc_comm_objs::PlaceFeedback feedback;
    mbzirc_comm_objs::PlaceResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");
      place_server_.setAborted(result);
      return;
    }

    ros::NodeHandle nh;
    ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");

    float z_offset = 0.6;  // TODO: offset in meters between uav and attached brick frames
    geometry_msgs::PoseStamped in_wall_uav_pose = _goal->in_wall_brick_pose;
    in_wall_uav_pose.pose.position.z += z_offset;

    ROS_INFO("in_wall_uav_pose = [%s][%lf, %lf, %lf][%lf, %lf, %lf, %lf]", in_wall_uav_pose.header.frame_id.c_str(), \
              in_wall_uav_pose.pose.position.x, in_wall_uav_pose.pose.position.y, in_wall_uav_pose.pose.position.z, \
              in_wall_uav_pose.pose.orientation.x, in_wall_uav_pose.pose.orientation.y, in_wall_uav_pose.pose.orientation.z, in_wall_uav_pose.pose.orientation.w);

    ual_->goToWaypoint(in_wall_uav_pose, true);
    ros::Duration(5.0).sleep();  // Give time to stabilize...

    mbzirc_comm_objs::Magnetize magnetize_srv;
    magnetize_srv.request.magnetize = false;
    if (!magnetize_client.call(magnetize_srv)) {
      ROS_ERROR("Failed to call [magnetize] service");  // TODO: retry?
    }

    place_server_.setSucceeded(result);
  }

  void landCallback(const mbzirc_comm_objs::LandGoalConstPtr &_goal) {
    // ROS_INFO("Land!");
    // mbzirc_comm_objs::LandFeedback feedback;
    mbzirc_comm_objs::LandResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");
      land_server_.setAborted(result);
      return;
    }

    ual_->land(true);

    land_server_.setSucceeded(result);
  }

  void moveInCirclesCallback(const mbzirc_comm_objs::MoveInCirclesGoalConstPtr &_goal) {
    mbzirc_comm_objs::MoveInCirclesFeedback feedback;
    mbzirc_comm_objs::MoveInCirclesResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      result.message = "UAL is not flying auto!";
      move_in_circles_server_.setAborted(result);
      return;
    }

    if (_goal->horizontal_distance < 1e-2) {
      result.message = "horizontal_distance must be greater than 1cm!";
      move_in_circles_server_.setAborted(result);
      return;
    }

    if (fabs(_goal->horizontal_velocity) < 1e-2) {
      result.message = "horizontal_velocity absolute value must be grater than 1cm/s!";
      move_in_circles_server_.setAborted(result);
      return;
    }

    geometry_msgs::TransformStamped robot_transform;
    std::string robot_frame_id = ual_->transform().child_frame_id;
    try {
      robot_transform = tf_buffer_.lookupTransform(_goal->origin.header.frame_id, robot_frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      result.message = ex.what();
      move_in_circles_server_.setAborted(result);
      return;
    }
    float delta_x = robot_transform.transform.translation.x - _goal->origin.pose.position.x;
    float delta_y = robot_transform.transform.translation.y - _goal->origin.pose.position.y;
    float theta = atan2(delta_y, delta_x);

    float frequency = 10;  // [Hz]
    ros::Rate loop_rate(frequency);
    float delta_theta = _goal->horizontal_velocity / (_goal->horizontal_distance * frequency);
    int max_iteration_count = _goal->revolutions_count * ceil(2*M_PI / fabs(delta_theta));
    int iteration_count = 0;

    while (ros::ok()) {
      if (move_in_circles_server_.isPreemptRequested()) {
        ual_->setPose(ual_->pose());
        move_in_circles_server_.setPreempted();
        return;
      }
      geometry_msgs::PoseStamped circle_reference = _goal->origin;
      circle_reference.pose.position.x += _goal->horizontal_distance * cos(theta);
      circle_reference.pose.position.y += _goal->horizontal_distance * sin(theta);
      circle_reference.pose.position.z += _goal->vertical_distance;
      float half_yaw = 0.5 * (theta + M_PI);
      circle_reference.pose.orientation.x = 0;
      circle_reference.pose.orientation.y = 0;
      circle_reference.pose.orientation.z = sin(half_yaw);
      circle_reference.pose.orientation.w = cos(half_yaw);
      if (iteration_count == 0) {
        ual_->goToWaypoint(circle_reference, true);
      } else {
        ual_->setPose(circle_reference);
      }
      feedback.current_target = circle_reference;
      move_in_circles_server_.publishFeedback(feedback);
      if (iteration_count++ >= max_iteration_count) { break; }
      theta += delta_theta;
      loop_rate.sleep();
    }
    move_in_circles_server_.setSucceeded(result);
  }

  void extinguishFacadeFireCallback(const mbzirc_comm_objs::ExtinguishFacadeFireGoalConstPtr &_goal) {
    mbzirc_comm_objs::ExtinguishFacadeFireFeedback feedback;
    mbzirc_comm_objs::ExtinguishFacadeFireResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      result.message = "UAL is not flying auto!";
      extinguish_facade_fire_server_.setAborted(result);
      return;
    }

    std::string fires_folder = ros::package::getPath("fire_extinguisher") + "/fires/";
    std::string fires_filename = fires_folder + (_goal->fires_file != ""? _goal->fires_file: "fire_default.yaml");

    std::map<std::string, FireData> fire_map;
    YAML::Node yaml_fires = YAML::LoadFile(fires_filename);  // TODO: Check file existence
    for (std::size_t i = 0; i < yaml_fires["fires"].size(); i++) {
        FireData fire_data;
        yaml_fires["fires"][i] >> fire_data;
        // std::cout << "\n index: " << i<< '\n';
        // std::cout << fire_data.id << '\n';
        // std::cout << fire_data.ual_pose << '\n';
        // std::cout << fire_data.sf11_range << '\n';
        // std::cout << fire_data.wall_list << '\n';
        fire_map[fire_data.id] = fire_data;
    }

    if (!fire_map.count(_goal->fire_id)) {
      result.message = "fire_id [" + _goal->fire_id + "] not found";
      extinguish_facade_fire_server_.setAborted(result);
      return;
    }
    FireData target_fire = fire_map[_goal->fire_id];

    ros::NodeHandle nh;
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
    ros::Subscriber walls_sub = nh.subscribe<mbzirc_comm_objs::WallList>("walls", 1, &UalActionServer::wallListCallback, this);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

    bool has_sf11_range;
    bool has_wall_list;
    for (int i = 0; i < 100; i++) {  // 100*0.1 = 10 seconds timeout
      has_sf11_range = (sf11_range_.header.seq != 0);
      has_wall_list = (wall_list_.header.seq != 0);
      if (has_sf11_range && has_wall_list) {
        break;
      }
      ros::Duration(0.1).sleep();
    }
    if (!has_sf11_range || !has_wall_list) {
      result.message = "could not get valid: ";
      if (!has_sf11_range) { result.message += "[sf11_range] "; }
      if (!has_wall_list) { result.message += "[wall_list]"; }
      extinguish_facade_fire_server_.setAborted(result);
      return;
    }

    grvc::ual::PIDParams pid_x;
    pid_x.kp = 0.4;
    pid_x.ki = 0.0;
    pid_x.kd = 0.0;
    pid_x.min_sat = -1.0;
    pid_x.max_sat = 1.0;
    pid_x.min_wind = -1.0;
    pid_x.max_wind = 1.0;

    grvc::ual::PIDParams pid_y;
    pid_y.kp = 0.4;
    pid_y.ki = 0.0;
    pid_y.kd = 0.0;
    pid_y.min_sat = -1.0;
    pid_y.max_sat = 1.0;
    pid_y.min_wind = -1.0;
    pid_y.max_wind = 1.0;

    grvc::ual::PIDParams pid_z;
    pid_z.kp = 0.4;
    pid_z.ki = 0.0;
    pid_z.kd = 0.0;
    pid_z.min_sat = -1.0;
    pid_z.max_sat = 1.0;
    pid_z.min_wind = -1.0;
    pid_z.max_wind = 1.0;

    grvc::ual::PIDParams pid_yaw;
    pid_yaw.kp = 0.4;
    pid_yaw.ki = 0.02;
    pid_yaw.kd = 0.0;
    pid_yaw.min_sat = -2.0;
    pid_yaw.max_sat = 2.0;
    pid_yaw.min_wind = -2.0;
    pid_yaw.max_wind = 2.0;
    pid_yaw.is_angular = true;

    grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
    pose_pid.enableRosInterface("extinguish_facade_control");

    // grvc::utils::PidController x_pid("x", 0.4, 0.02, 0);
    // grvc::utils::PidController y_pid("y", 0.4, 0.02, 0);
    // grvc::utils::PidController z_pid("z", 0.4, 0.02, 0);
    // grvc::utils::PidController yaw_pid("yaw", 0.4, 0.02, 0);

    mbzirc_comm_objs::Wall fire_closest_wall = closestWall(target_fire.wall_list);
    mbzirc_comm_objs::Wall fire_largest_wall = largestWall(target_fire.wall_list);
    double fire_ual_yaw = 2 * atan2(target_fire.ual_pose.pose.orientation.z, target_fire.ual_pose.pose.orientation.w);

    ros::Rate loop_rate(EXTINGUISH_LOOP_RATE);
    while (ros::ok()) {
      if (extinguish_facade_fire_server_.isPreemptRequested()) {
        ual_->setPose(ual_->pose());
        extinguish_facade_fire_server_.setPreempted();
        return;
      }

      double z_error = target_fire.sf11_range.range - sf11_range_.range;
      // ROS_ERROR("z: %lf", z_error);

      // TODO: target criteria
      mbzirc_comm_objs::Wall target_wall = fire_closest_wall;
      mbzirc_comm_objs::Wall current_wall = closestWall(wall_list_);
      // mbzirc_comm_objs::Wall target_wall = fire_largest_wall;
      // mbzirc_comm_objs::Wall current_wall = largestWall(wall_list_);

      visualization_msgs::MarkerArray marker_array;
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0;
      marker_array.markers.push_back(getLineMarker(target_wall, wall_list_.header.frame_id, color, 0, "target_wall"));

      double x_error = 0;
      double y_error = 0;
      double squared_delta_length = fabs(squaredLength(target_wall) - squaredLength(current_wall));
      if (squared_delta_length < 10.0) {  // TODO: Tune threshold (sq!)
          double x_start_error = target_wall.start[0] - current_wall.start[0];
          double y_start_error = target_wall.start[1] - current_wall.start[1];
          double x_end_error = target_wall.end[0] - current_wall.end[0];
          double y_end_error = target_wall.end[1] - current_wall.end[1];
          x_error = 0.5 * (x_start_error + x_end_error);
          y_error = 0.5 * (y_start_error + y_end_error);
          // ROS_ERROR("xy: %lf, %lf", x_error, y_error);
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 1.0;
      } else {
          // ROS_ERROR("squared_delta_length = %lf", squared_delta_length);
          // std::cout << target_wall;
          // std::cout << current_wall;
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 1.0;
      }
      marker_array.markers.push_back(getLineMarker(current_wall, wall_list_.header.frame_id, color, 1, "target_wall"));
      marker_pub_.publish(marker_array);  // TODO: optional!

      // ROS_ERROR("start: %lf, %lf", x_start_error, y_start_error);
      // ROS_ERROR("end: %lf, %lf", x_end_error, y_end_error);
      // ROS_ERROR("xy: %lf, %lf", x_error, y_error);
      // std::cout << current_wall;

      auto ual_pose = ual_->pose();
      double current_yaw = 2 * atan2(ual_pose.pose.orientation.z, ual_pose.pose.orientation.w);
      double yaw_error = fire_ual_yaw - current_yaw;

      // geometry_msgs::TwistStamped velocity;  // TODO: frame_id?
      // velocity.header.stamp = ros::Time::now();
      // velocity.header.frame_id = wall_list_.header.frame_id;
      geometry_msgs::PoseStamped error_pose;
      error_pose.header.stamp = ros::Time::now();
      error_pose.header.frame_id = wall_list_.header.frame_id;
      error_pose.pose.position.x = -x_error;
      error_pose.pose.position.y = -y_error;
      error_pose.pose.position.z = z_error;
      error_pose.pose.orientation.x = 0;
      error_pose.pose.orientation.y = 0;
      error_pose.pose.orientation.z = sin(0.5 * yaw_error);
      error_pose.pose.orientation.w = cos(0.5 * yaw_error);
      // velocity.twist.linear.x = x_pid.control_signal(-x_error, 1.0/EXTINGUISH_LOOP_RATE);
      // velocity.twist.linear.y = y_pid.control_signal(-y_error, 1.0/EXTINGUISH_LOOP_RATE);
      // velocity.twist.linear.z = z_pid.control_signal(z_error, 1.0/EXTINGUISH_LOOP_RATE);
      // velocity.twist.angular.z = yaw_pid.control_signal(yaw_error, 1.0/EXTINGUISH_LOOP_RATE);
      geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);

      // std::cout << velocity << '\n';
      ual_->setVelocity(velocity);

      loop_rate.sleep();
    }
  }

  void extinguishGroundFireCallback(const mbzirc_comm_objs::ExtinguishGroundFireGoalConstPtr &_goal) {
    mbzirc_comm_objs::ExtinguishGroundFireFeedback feedback;
    mbzirc_comm_objs::ExtinguishGroundFireResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      result.message = "UAL is not flying auto!";
      extinguish_ground_fire_server_.setAborted(result);
      return;
    }

    ros::NodeHandle nh;
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
    // TODO: Add other subscribers (e.g. fire estimation)
    // TODO: Markers?
    //ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

    bool has_sf11_range = false;
    bool has_fire_estimation = false;
    for (int i = 0; i < 100; i++) {  // 100*0.1 = 10 seconds timeout
      has_sf11_range = (sf11_range_.header.seq != 0);
      has_fire_estimation = true; // TODO: Add real condition
      if (has_sf11_range && has_fire_estimation) {
        break;
      }
      ros::Duration(0.1).sleep();
    }
    if (!has_sf11_range || !has_fire_estimation) {
      result.message = "could not get valid: ";
      if (!has_sf11_range) { result.message += "[sf11_range] "; }
      if (!has_fire_estimation) { result.message += "[fire_estimation] "; }
      extinguish_ground_fire_server_.setAborted(result);
      return;
    }

    grvc::ual::PIDParams pid_x;
    pid_x.kp = 0.4;
    pid_x.ki = 0.0;
    pid_x.kd = 0.0;
    pid_x.min_sat = -1.0;
    pid_x.max_sat = 1.0;
    pid_x.min_wind = -1.0;
    pid_x.max_wind = 1.0;

    grvc::ual::PIDParams pid_y;
    pid_y.kp = 0.4;
    pid_y.ki = 0.0;
    pid_y.kd = 0.0;
    pid_y.min_sat = -1.0;
    pid_y.max_sat = 1.0;
    pid_y.min_wind = -1.0;
    pid_y.max_wind = 1.0;

    grvc::ual::PIDParams pid_z;
    pid_z.kp = 0.4;
    pid_z.ki = 0.0;
    pid_z.kd = 0.0;
    pid_z.min_sat = -1.0;
    pid_z.max_sat = 1.0;
    pid_z.min_wind = -1.0;
    pid_z.max_wind = 1.0;

    grvc::ual::PIDParams pid_yaw;
    pid_yaw.kp = 0.4;
    pid_yaw.ki = 0.02;
    pid_yaw.kd = 0.0;
    pid_yaw.min_sat = -2.0;
    pid_yaw.max_sat = 2.0;
    pid_yaw.min_wind = -2.0;
    pid_yaw.max_wind = 2.0;
    pid_yaw.is_angular = true;

    grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
    pose_pid.enableRosInterface("extinguish_ground_control");

    // TODO: Get fire orientation from estimation
    //double fire_ual_yaw = 2 * atan2(target_fire.ual_pose.pose.orientation.z, target_fire.ual_pose.pose.orientation.w);

    ros::Rate loop_rate(EXTINGUISH_LOOP_RATE);
    while (ros::ok()) {
      if (extinguish_ground_fire_server_.isPreemptRequested()) {
        ual_->setPose(ual_->pose());
        extinguish_ground_fire_server_.setPreempted();
        return;
      }

      double x_error, y_error, z_error, yaw_error;

      // TODO: Get z_error
      //z_error = target_fire.sf11_range.range - sf11_range_.range;
      // ROS_ERROR("z: %lf", z_error);

      auto ual_pose = ual_->pose();
      double current_yaw = 2 * atan2(ual_pose.pose.orientation.z, ual_pose.pose.orientation.w);
      // TODO: Get yaw_error
      //yaw_error = fire_ual_yaw - current_yaw;

      // ----------------------------------------------------------//
      // Put errors yo 0 until implementation is done (To be removed)
      x_error = 0;
      y_error = 0;
      z_error = 0;
      yaw_error = 0;
      // ----------------------------------------------------------//

      geometry_msgs::PoseStamped error_pose;
      error_pose.header.stamp = ros::Time::now();
      // TODO: set frame
      //error_pose.header.frame_id = XXX.header.frame_id;
      error_pose.pose.position.x = -x_error;
      error_pose.pose.position.y = -y_error;
      error_pose.pose.position.z = z_error;
      error_pose.pose.orientation.x = 0;
      error_pose.pose.orientation.y = 0;
      error_pose.pose.orientation.z = sin(0.5 * yaw_error);
      error_pose.pose.orientation.w = cos(0.5 * yaw_error);
      geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);
      //ual_->setVelocity(velocity); // TODO: Uncomment when implemented

      loop_rate.sleep();
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_action_server");

  UalActionServer uav_action_server;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
