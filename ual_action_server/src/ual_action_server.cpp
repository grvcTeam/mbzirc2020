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
#include <mbzirc_comm_objs/HoverAction.h>
#include <mbzirc_comm_objs/GoToAction.h>
#include <mbzirc_comm_objs/FollowPathAction.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/PickAction.h>
#include <uav_abstraction_layer/State.h>

class HistoryBuffer {  // TODO: template? utils?
public:
  void set_size(size_t _size) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_size_ = _size;
    buffer_.clear();
    current_ = 0;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
    current_ = 0;
  }

  void update(double _value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.size() < buffer_size_) {
      buffer_.push_back(_value);
    } else {
      buffer_[current_] = _value;
      current_ = (current_ + 1) % buffer_size_;
    }
  }

  bool get_stats(double& _min, double& _mean, double& _max) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.size() >= buffer_size_) {
      double min_value = +std::numeric_limits<double>::max();
      double max_value = -std::numeric_limits<double>::max();
      double sum = 0;
      for (int i = 0; i < buffer_.size(); i++) {
        if (buffer_[i] < min_value) { min_value = buffer_[i]; }
        if (buffer_[i] > max_value) { max_value = buffer_[i]; }
        sum += buffer_[i];
      }
      _min = min_value;
      _max = max_value;
      _mean = sum / buffer_.size();
      return true;
    }
      return false;
  }

  bool get_variance(double& _var) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.size() >= buffer_size_) {
      double mean = 0;
      double sum = 0;
      _var = 0;
      for (int i = 0; i < buffer_.size(); i++) {
        sum += buffer_[i];
      }
      mean = sum / buffer_.size();
      for (int i = 0; i < buffer_.size(); i++) {
        _var += (buffer_[i]-mean)*(buffer_[i]-mean);
      }
      return true;
    }
    return false;
  }

protected:
  size_t buffer_size_ = 0;
  unsigned int current_ = 0;
  std::vector<double> buffer_;
  std::mutex mutex_;
};


class UalActionServer {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::HoverAction> hover_server_;  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<mbzirc_comm_objs::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::FollowPathAction> follow_path_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::PickAction> pick_server_;
  std::string robot_id_;  // TODO: Used?
  grvc::ual::UAL *ual_;
  mbzirc_comm_objs::ObjectDetection matched_candidate_;

public:

  UalActionServer(std::string _robot_id):
    hover_server_(nh_, "hover_action", boost::bind(&UalActionServer::hoverCallback, this, _1), false),
    go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
    follow_path_server_(nh_, "follow_path_action", boost::bind(&UalActionServer::followPathCallback, this, _1), false),
    pick_server_(nh_, "pick_action", boost::bind(&UalActionServer::pickCallback, this, _1), false),
    robot_id_(_robot_id) {

    // ros::param::set("~uav_id", _robot_id);
    // ros::param::set("~pose_frame_id", "map");
    ual_ = new grvc::ual::UAL();
    hover_server_.start();
    go_to_server_.start();
    follow_path_server_.start();
    pick_server_.start();
  }

  ~UalActionServer() {
    delete ual_;
  }

  void hoverCallback(const mbzirc_comm_objs::HoverGoalConstPtr &_goal) {
    // ROS_INFO("Hover!");
    // mbzirc_comm_objs::HoverFeedback feedback;
    mbzirc_comm_objs::HoverResult result;

    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");  // ROS_WARN("UAL %d is uninitialized!", uav_id);
      sleep(1);
    }
    // TODO: Fill result
    switch (ual_->state().state) {
      case uav_abstraction_layer::State::LANDED_DISARMED:
        ROS_WARN("UAL is disarmed!");
        hover_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDED_ARMED:
        // TODO: from param, substract current z
        ual_->takeOff(_goal->height, true);  // TODO: timeout? preempt?
        hover_server_.setSucceeded(result);
        break;
      case uav_abstraction_layer::State::TAKING_OFF:
        ROS_WARN("UAL is taking off!");
        // TODO: Wait until FLYING_AUTO?
        hover_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::FLYING_AUTO:
        // TODO: goto current x,y, but z = flight_level?
        hover_server_.setSucceeded(result);
        break;
      case uav_abstraction_layer::State::FLYING_MANUAL:
        ROS_WARN("UAL is flying manual!");
        hover_server_.setAborted(result);
        break;
      case uav_abstraction_layer::State::LANDING:
        ROS_WARN("UAL is landing!");
        // TODO: Wait until LANDED_ARMED and then take off?
        hover_server_.setAborted(result);
        break;
      default:
        ROS_ERROR("Unexpected UAL state!");
        hover_server_.setAborted(result);
    }
  }

  void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal) {
    // ROS_INFO("Go to!");
    // mbzirc_comm_objs::GoToFeedback feedback;
    mbzirc_comm_objs::GoToResult result;

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
    if (msg->objects.size() > 0) {
      matched_candidate_ = msg->objects[0];  // TODO: Something better than grabbing first found?
      ROS_INFO("Candidate relative position = [%lf, %lf, %lf]", matched_candidate_.relative_position.x, matched_candidate_.relative_position.y, matched_candidate_.relative_position.z);
    }
  }

  void pickCallback(const mbzirc_comm_objs::PickGoalConstPtr &_goal) {
    // ROS_INFO("Pick!");
    // mbzirc_comm_objs::PickFeedback feedback;
    mbzirc_comm_objs::PickResult result;

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");  // ROS_WARN("UAL %d is not flying auto!", uav_id);
      pick_server_.setAborted(result);
      return;
    }

    ros::NodeHandle nh;
    ros::Subscriber sensed_sub_ = nh.subscribe("sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);
    ros::Duration(1.0).sleep();  // TODO: tune!

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #define Z_GIVE_UP_CATCHING 17.5  // TODO: From config file?
    #define Z_RETRY_CATCH 1.0
    #define CANDIDATE_TIMEOUT 1.5  // [s]
    #define CATCHING_LOOP_RATE 10  // [Hz]
    #define AVG_XY_ERROR_WINDOW_SIZE 13
    #define MAX_DELTA_Z 0.5  // [m] --> [m/s]
    #define MAX_AVG_XY_ERROR 0.3  // [m]

    // TODO: Magnetize catching device
    // catching_device_->setMagnetization(true);
    
    // HistoryBuffer history_xy_errors;
    // history_xy_errors.set_size(AVG_XY_ERROR_WINDOW_SIZE);
    std::vector<double> history_xy_errors(AVG_XY_ERROR_WINDOW_SIZE, MAX_AVG_XY_ERROR);
    unsigned tries_counter = 0;
    ros::Duration timeout(CANDIDATE_TIMEOUT);
    ros::Rate loop_rate(CATCHING_LOOP_RATE);
    geometry_msgs::Point target_position = matched_candidate_.relative_position;
    while (true) {
      ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
      ROS_INFO("since_last_candidate = %lf, timeout = %lf", since_last_candidate.toSec(), timeout.toSec());

      if (since_last_candidate < timeout) {
        // x-y-control: in candidateCallback
        // z-control: descend
        double xy_error = sqrt(target_position.x*target_position.x + target_position.y*target_position.y);
        history_xy_errors.push_back(xy_error);
        if (history_xy_errors.size() > AVG_XY_ERROR_WINDOW_SIZE) {  // 666 TODO: Check window size for avg xy error!
          history_xy_errors.erase(history_xy_errors.begin());
        }

        double avg_xy_error = std::accumulate(history_xy_errors.begin(), history_xy_errors.end(), 0.0) / static_cast<float>(history_xy_errors.size());
        if (avg_xy_error > MAX_AVG_XY_ERROR) {
          avg_xy_error = MAX_AVG_XY_ERROR;
        }
        target_position.z = -MAX_DELTA_Z * (1.0 - (avg_xy_error / MAX_AVG_XY_ERROR));
        ROS_INFO("xy_error = %lf, avg_xy_error = %lf, target_position.z = %lf", xy_error, avg_xy_error, target_position.z);

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
      // grvc::ual::PositionError error;  
      // error.vector.x = target_position.x;
      // error.vector.y = target_position.y;
      // error.vector.z = target_position.z;
      // ual_.setPositionError(error);
      ROS_INFO("target_position = [%lf, %lf, %lf] target angle = x", target_position.x, target_position.y, target_position.z);

      // TODO: Look for equivalent!
      // if (catching_device_->switchIsPressed()) {
      //   pick_server_.setSucceeded(result);
      //   break;
      // }

      // If we're too high, give up
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

/*
  void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal) {
    ROS_INFO("Go to!");
    // mbzirc_comm_objs::GoToFeedback feedback;
    mbzirc_comm_objs::GoToResult result;
    // TODO: Fill result

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");  // ROS_WARN("UAL %d is not flying auto!", uav_id);
      go_to_server_.setAborted(result);
    }

    ual_->goToWaypoint(_goal->waypoint, true);  // TODO: blocking? preemptable?
    go_to_server_.setSucceeded(result);
  }
 */

  void followPathCallback(const mbzirc_comm_objs::FollowPathGoalConstPtr &_goal) {
    // ROS_INFO("Follow path!");  // TODO: check for collisions?
    // mbzirc_comm_objs::GoToFeedback feedback;
    mbzirc_comm_objs::FollowPathResult result;
    // TODO: Fill result

    if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
      ROS_WARN("UAL is not flying auto!");  // ROS_WARN("UAL %d is not flying auto!", uav_id);
      follow_path_server_.setAborted(result);
    }

    // for (size_t i = 0; i < _goal->path.size(); i++) {
    //   ual_->goToWaypoint(_goal->path[i], true);  // TODO: blocking? preemptable?
    // }
    for (auto waypoint: _goal->path) {
      ual_->goToWaypoint(waypoint, true);  // TODO: blocking? preemptable?
    }

    follow_path_server_.setSucceeded(result);
  }

  /*
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    */
  // }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ual_action_server");

  UalActionServer ual_action_server("");  // Not needed?
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  // ros::spin();

  return 0;
}
