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
#include <uav_abstraction_layer/State.h>

class UalActionServer {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::HoverAction> hover_server_;  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<mbzirc_comm_objs::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::FollowPathAction> follow_path_server_;
  std::string robot_id_;  // TODO: Used?
  grvc::ual::UAL *ual_;

public:

  UalActionServer(std::string _robot_id):
    hover_server_(nh_, "hover_action", boost::bind(&UalActionServer::hoverCallback, this, _1), false),
    go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
    follow_path_server_(nh_, "follow_path_action", boost::bind(&UalActionServer::followPathCallback, this, _1), false),
    robot_id_(_robot_id) {
    ual_ = new grvc::ual::UAL();
    hover_server_.start();
    go_to_server_.start();
    follow_path_server_.start();
  }

  ~UalActionServer() {
    delete ual_;
  }

  void hoverCallback(const mbzirc_comm_objs::HoverGoalConstPtr &_goal) {
    ROS_INFO("Hover!");
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

  void followPathCallback(const mbzirc_comm_objs::FollowPathGoalConstPtr &_goal) {
    ROS_INFO("Follow path!");  // TODO: check for collisions?
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
