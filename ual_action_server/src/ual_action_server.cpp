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
#include <actionlib/server/simple_action_server.h>
#include <uav_abstraction_layer/ual.h>
#include <ual_action_server/TakeOffAction.h>
#include <ual_action_server/GoToAction.h>
#include <ual_action_server/PickAction.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/GripperAttached.h>
#include <mbzirc_comm_objs/Magnetize.h>
#include <uav_abstraction_layer/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/pid_controller.h>
#include <handy_tools/circular_buffer.h>

class UalActionServer {
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs
  actionlib::SimpleActionServer<ual_action_server::TakeOffAction> take_off_server_;
  actionlib::SimpleActionServer<ual_action_server::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<ual_action_server::PickAction> pick_server_;
  std::string robot_id_;  // TODO: Used?
  grvc::ual::UAL *ual_;
  mbzirc_comm_objs::ObjectDetection matched_candidate_;
  bool gripper_attached_ = false;

public:

  UalActionServer(std::string _robot_id):
    take_off_server_(nh_, "take_off_action", boost::bind(&UalActionServer::takeOffCallback, this, _1), false),
    go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
    pick_server_(nh_, "pick_action", boost::bind(&UalActionServer::pickCallback, this, _1), false),
    robot_id_(_robot_id) {

    // ros::param::set("~uav_id", _robot_id);
    // ros::param::set("~pose_frame_id", "map");
    ual_ = new grvc::ual::UAL();
    take_off_server_.start();
    go_to_server_.start();
    pick_server_.start();
  }

  ~UalActionServer() {
    delete ual_;
  }

  // TODO: Who should know about flight_level? ual_action_server, uav_agent, central_agent...
  void takeOffCallback(const ual_action_server::TakeOffGoalConstPtr &_goal) {
    // ROS_INFO("Take off!");
    // ual_action_server::TakeOffFeedback feedback;
    ual_action_server::TakeOffResult result;

    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");  // ROS_WARN("UAL %d is uninitialized!", uav_id);
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

  void goToCallback(const ual_action_server::GoToGoalConstPtr &_goal) {
    // ROS_INFO("Go to!");
    // ual_action_server::GoToFeedback feedback;
    ual_action_server::GoToResult result;

    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");  // ROS_WARN("UAL %d is uninitialized!", uav_id);
      sleep(1);
    }
    // TODO: Fill result
    switch (ual_->state().state) {
      case uav_abstraction_layer::State::LANDED_DISARMED:
        ROS_WARN("UAL is disarmed!");
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDED_ARMED:
        ual_->takeOff(_goal->waypoint.pose.position.z - ual_->pose().pose.position.z, true);  // TODO: timeout? preempt?
        ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt?
        go_to_server_.setSucceeded(result);
        break;
      case uav_abstraction_layer::State::TAKING_OFF:
        ROS_WARN("UAL is taking off!");
        // TODO: Wait until FLYING_AUTO?
        go_to_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::FLYING_AUTO:
        ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt?
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

  void sensedObjectsCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) {
    float min_distance = 1e3;  // 1 Km
    for (auto object: msg->objects) {
      float distance = object.relative_position.x * object.relative_position.x + object.relative_position.y * object.relative_position.y;
      if (distance < min_distance) {
        min_distance = distance;
        matched_candidate_ = object;  // TODO: Check also color is correct
      }
    }
  }

  void attachedCallback(const mbzirc_comm_objs::GripperAttachedConstPtr& msg) {
    gripper_attached_ = msg->attached;
  }

  void pickCallback(const ual_action_server::PickGoalConstPtr &_goal) {
    // ROS_INFO("Pick!");
    // ual_action_server::PickFeedback feedback;
    ual_action_server::PickResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");  // ROS_WARN("UAL %d is not flying auto!", uav_id);
      pick_server_.setAborted(result);
      return;
    }

    ros::NodeHandle nh;
    ros::Subscriber sensed_sub_ = nh.subscribe("sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);
    ros::Subscriber attached_sub_ = nh.subscribe("attached", 1, &UalActionServer::attachedCallback, this);
    ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");
    ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Tune!
    #define Z_GIVE_UP_CATCHING 17.5  // TODO: From config file?
    #define Z_RETRY_CATCH 1.0
    #define CANDIDATE_TIMEOUT 1.5  // [s]
    #define CATCHING_LOOP_RATE 10  // [Hz]
    #define AVG_XY_ERROR_WINDOW_SIZE 13
    #define MAX_DELTA_Z 0.5  // [m] --> [m/s]
    #define MAX_AVG_XY_ERROR 0.3  // [m]

    grvc::utils::PidController x_pid("x", 0.4, 0.02, 0);  // TODO: class members? 
    grvc::utils::PidController y_pid("y", 0.4, 0.02, 0);
    grvc::utils::PidController z_pid("z", 0.4, 0.02, 0);

    // TODO: Magnetize catching device
    // catching_device_->setMagnetization(true);
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
    while (true) {
      ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
      // ROS_INFO("since_last_candidate = %lf, timeout = %lf", since_last_candidate.toSec(), timeout.toSec());

      geometry_msgs::Point target_position = matched_candidate_.relative_position;
      if (since_last_candidate < timeout) {
        // x-y-control: in candidateCallback
        // z-control: descend
        double xy_error = sqrt(target_position.x*target_position.x + target_position.y*target_position.y);
        history_xy_errors.push(xy_error);
        double min_xy_error, avg_xy_error, max_xy_error;
        history_xy_errors.get_stats(min_xy_error, avg_xy_error, max_xy_error);

        if (avg_xy_error > MAX_AVG_XY_ERROR) {
          avg_xy_error = MAX_AVG_XY_ERROR;
        }
        target_position.z = -MAX_DELTA_Z * (1.0 - (avg_xy_error / MAX_AVG_XY_ERROR));
        // ROS_INFO("xy_error = %lf, avg_xy_error = %lf, target_position.z = %lf", xy_error, avg_xy_error, target_position.z);

      } else {  // No fresh candidates (timeout)
        ROS_WARN("Timeout!");

        // TODO: Push MAX_AVG_XY_ERROR into history_xy_errors
        target_position.x = 0.0;
        target_position.y = 0.0;
        target_position.z = MAX_DELTA_Z;

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
      geometry_msgs::TwistStamped velocity;
      velocity.header.stamp = ros::Time::now();
      velocity.header.frame_id = "map";
      velocity.twist.linear.x = x_pid.control_signal(target_position.x, 1.0 / CATCHING_LOOP_RATE);
      velocity.twist.linear.y = y_pid.control_signal(target_position.y, 1.0 / CATCHING_LOOP_RATE);
      velocity.twist.linear.z = z_pid.control_signal(target_position.z, 1.0 / CATCHING_LOOP_RATE);

      ual_->setVelocity(velocity);
      // ROS_INFO("Candidate relative position = [%lf, %lf, %lf]", matched_candidate_.relative_position.x, matched_candidate_.relative_position.y, matched_candidate_.relative_position.z);
      // ROS_INFO("target_position = [%lf, %lf, %lf] target angle = x", target_position.x, target_position.y, target_position.z);
      // ROS_INFO("velocity = [%lf, %lf, %lf]", velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);

      // TODO: Look for equivalent!
      // if (catching_device_->switchIsPressed()) {
      if (gripper_attached_) {
        pick_server_.setSucceeded(result);
        break;
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
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ual_action_server");

  UalActionServer ual_action_server("");  // Not needed?
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
