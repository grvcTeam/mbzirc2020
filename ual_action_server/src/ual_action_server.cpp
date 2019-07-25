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
#include <mbzirc_comm_objs/UALAction.h>
#include <uav_abstraction_layer/State.h>

class UALAction {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::UALAction> as_;  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string robot_id_;
  grvc::ual::UAL *ual_;
  // create messages that are used to published feedback/result
  mbzirc_comm_objs::UALFeedback feedback_;
  mbzirc_comm_objs::UALResult result_;

public:

  UALAction(std::string _robot_id):
    as_(nh_, "ual_action", boost::bind(&UALAction::executeCallback, this, _1), false),
    robot_id_(_robot_id) {
    ual_ = new grvc::ual::UAL();
    as_.start();
  }

  ~UALAction() {
    delete ual_;
  }

  void hover(double _flight_level) {
    while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok()) {
      ROS_WARN("UAL is uninitialized!");  // ROS_WARN("UAL %d is uninitialized!", uav_id);
      sleep(1);
    }
    switch (ual_->state().state) {
      case uav_abstraction_layer::State::LANDED_DISARMED:
        ROS_WARN("UAL is disarmed!");
        as_.setAborted(result_);
        break;
      case uav_abstraction_layer::State::LANDED_ARMED:
        ual_->takeOff(_flight_level, true);
        as_.setSucceeded(result_);
        break;
      case uav_abstraction_layer::State::TAKING_OFF:
        ROS_WARN("UAL is taking off!");
        as_.setAborted(result_);
        break;
      case uav_abstraction_layer::State::FLYING_AUTO:
        // TODO: goto current x,y, but z = flight_level
        as_.setSucceeded(result_);
        break;
      case uav_abstraction_layer::State::FLYING_MANUAL:
        ROS_WARN("UAL is flying manual!");
        as_.setAborted(result_);
        break;
      case uav_abstraction_layer::State::LANDING:
        ROS_WARN("UAL is landing!");
        as_.setAborted(result_);
        break;
      default:
        ROS_ERROR("Unexpected UAL state!");
        as_.setAborted(result_);
    }
  }

  void executeCallback(const mbzirc_comm_objs::UALGoalConstPtr &_goal) {
    double flight_level = 2.0;  // TODO: from param, substract current z
    // TODO: Fill result?
    switch(_goal->command) {
      case mbzirc_comm_objs::UALGoal::HOVER:
        ROS_INFO("HOVER command received");
        hover(flight_level);
        break;
      case mbzirc_comm_objs::UALGoal::GOTO:
        ROS_INFO("GOTO command received");
        as_.setSucceeded(result_);
        break;
      case mbzirc_comm_objs::UALGoal::PICK:
        ROS_INFO("PICK command received");
        as_.setSucceeded(result_);
        break;
      default:
        ROS_ERROR("Unexpected UALGoal command [%d]!", _goal->command);
        as_.setAborted(result_);
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
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ual_action_server");

  UALAction ual_action("");  // Not needed?
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  // ros::spin();

  return 0;
}
