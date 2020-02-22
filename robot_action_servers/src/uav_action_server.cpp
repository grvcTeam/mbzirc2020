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
#include <robot_action_servers/uav_action_server.h>
#include <uav_abstraction_layer/ual_backend_dummy.h>
#include <ual_backend_mavros/ual_backend_mavros.h>
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>
#include <uav_abstraction_layer/State.h>

UalActionServer::UalActionServer():
  take_off_server_(nh_, "take_off_action", boost::bind(&UalActionServer::takeOffCallback, this, _1), false),
  go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
  pick_server_(nh_, "pick_action", boost::bind(&UalActionServer::pickCallback, this, _1), false),
  place_server_(nh_, "place_action", boost::bind(&UalActionServer::placeCallback, this, _1), false),
  land_server_(nh_, "land_action", boost::bind(&UalActionServer::landCallback, this, _1), false),
  move_in_circles_server_(nh_, "move_in_circles_action", boost::bind(&UalActionServer::moveInCirclesCallback, this, _1), false),
  extinguish_facade_fire_server_(nh_, "extinguish_facade_fire_action", boost::bind(&UalActionServer::extinguishFacadeFireCallback, this, _1), false),
  extinguish_ground_fire_server_(nh_, "extinguish_ground_fire_action", boost::bind(&UalActionServer::extinguishGroundFireCallback, this, _1), false),
  look_for_ground_fires_server_(nh_, "look_for_ground_fires_action", boost::bind(&UalActionServer::lookForGroundFiresCallback, this, _1), false) {

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
  look_for_ground_fires_server_.start();
}

UalActionServer::~UalActionServer() {
  delete ual_;
}

void UalActionServer::publishDataFeed(const ros::TimerEvent&) {
  mbzirc_comm_objs::RobotDataFeed data_feed;
  data_feed.pose = ual_->pose();
  data_feed_pub_.publish(data_feed);
}

bool UalActionServer::waitForFreshMatchedCandidateMsg(uint8_t seconds) {
  matched_candidate_.header.seq = 0;
  for (int i = 0; i < 10 * seconds; i++) {
    ros::Duration(0.1).sleep();
    if (matched_candidate_.header.seq != 0) {
      return true;
    }
  }
  return false;
}

bool UalActionServer::waitForFreshGripperAttachedMsg(uint8_t seconds) {
  gripper_attached_.header.seq = 0;
  for (int i = 0; i < 10 * seconds; i++) {
    ros::Duration(0.1).sleep();
    if (gripper_attached_.header.seq != 0) {
      return true;
    }
  }
  return false;
}

bool UalActionServer::waitForFreshSf11RangeMsg(uint8_t seconds) {
  sf11_range_.header.seq = 0;
  for (int i = 0; i < 10 * seconds; i++) {
    ros::Duration(0.1).sleep();
    if (sf11_range_.header.seq != 0) {
      return true;
    }
  }
  return false;
}

bool UalActionServer::waitForFreshWallListMsg(uint8_t seconds) {
  wall_list_.header.seq = 0;
  for (int i = 0; i < 10 * seconds; i++) {
    ros::Duration(0.1).sleep();
    if (wall_list_.header.seq != 0) {
      return true;
    }
  }
  return false;
}

// TODO: Who should know about flight_level? robot_action_servers, uav_agent, central_agent...
void UalActionServer::takeOffCallback(const mbzirc_comm_objs::TakeOffGoalConstPtr &_goal) {
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
      ual_->setHome(true);
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

void UalActionServer::landCallback(const mbzirc_comm_objs::LandGoalConstPtr &_goal) {
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

void UalActionServer::goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal) {
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

void UalActionServer::trackedBrickCallback(const mbzirc_comm_objs::ObjectDetectionConstPtr& msg) {
  if (msg->type != mbzirc_comm_objs::ObjectDetection::TYPE_BRICK_TRACK) {
    ROS_WARN("Expected TYPE_BRICK_TRACK!");
    return;
  }
  matched_candidate_ = *msg;  // TODO: Check also color is correct?
}

void UalActionServer::trackedFireCallback(const mbzirc_comm_objs::ObjectDetectionConstPtr& msg) {
  if (msg->type != mbzirc_comm_objs::ObjectDetection::TYPE_FIRE) {
    ROS_WARN("Expected TYPE_FIRE!");
    return;
  }
  matched_candidate_ = *msg;
}

void UalActionServer::sensedObjectsCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) {
  sensed_objects_ = *msg;
}

void UalActionServer::sf11RangeCallback(const sensor_msgs::RangeConstPtr& msg) {
  sf11_range_ = *msg;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_action_server");

  UalActionServer uav_action_server;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
