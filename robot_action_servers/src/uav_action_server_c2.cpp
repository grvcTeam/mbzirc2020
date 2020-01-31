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
#include <robot_action_servers/uav_action_server.h>

void UalActionServer::attachedCallback(const mbzirc_comm_objs::GripperAttachedConstPtr& msg) {
  gripper_attached_ = msg->attached;
}

void UalActionServer::pickCallback(const mbzirc_comm_objs::PickGoalConstPtr &_goal) {
  // ROS_INFO("Pick!");
  // mbzirc_comm_objs::PickFeedback feedback;
  mbzirc_comm_objs::PickResult result;

  if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
    ROS_WARN("UAL is not flying auto!");
    pick_server_.setAborted(result);
    return;
  }

  geometry_msgs::TransformStamped optical_to_camera_control;  // Constant!
  try {
    optical_to_camera_control = tf_buffer_.lookupTransform(tf_prefix_ + "/camera_control_link", tf_prefix_ + "/camera_color_optical_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  ros::NodeHandle nh;
  ros::Subscriber sensed_sub = nh.subscribe("tracked_object", 1, &UalActionServer::trackedObjectCallback, this);
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
  ros::Subscriber attached_sub = nh.subscribe<mbzirc_comm_objs::GripperAttached>("actuators_system/gripper_attached", 1, &UalActionServer::attachedCallback, this);
  ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");  // TODO: (only sim)
  ros::ServiceClient open_gripper_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/open_gripper");
  ros::ServiceClient close_gripper_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/close_gripper");
  ros::ServiceClient magnetize_gripper_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/magnetize_gripper");
  ros::ServiceClient demagnetize_gripper_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/demagnetize_gripper");
  ros::ServiceClient set_detection_client = nh.serviceClient<mbzirc_comm_objs::DetectTypes>("set_types");
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
  grvc::ual::PIDParams pid_x;
  pid_x.kp = 0.92;
  pid_x.ki = 0.0;
  pid_x.kd = 0.0;
  pid_x.min_sat = -2.0;
  pid_x.max_sat = 2.0;
  pid_x.min_wind = -2.0;
  pid_x.max_wind = 2.0;

  grvc::ual::PIDParams pid_y;
  pid_y.kp = 0.92;
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
  pid_yaw.ki = 0.04;
  pid_yaw.kd = 0.0;
  pid_yaw.min_sat = -2.0;
  pid_yaw.max_sat = 2.0;
  pid_yaw.min_wind = -2.0;
  pid_yaw.max_wind = 2.0;
  pid_yaw.is_angular = true;

  grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
  pose_pid.enableRosInterface("pick_control");  // TODO: disable as PID is tuned!
  double absolute_yaw_lock = 0;

  // TODO: Magnetize catching device
  mbzirc_comm_objs::Magnetize magnetize_srv;
  magnetize_srv.request.magnetize = true;
  if (!magnetize_client.call(magnetize_srv)) {
    ROS_ERROR("Failed to call (sim) magnetize service");
  }
  std_srvs::Trigger trigger;
  if (!magnetize_gripper_client.call(trigger)) {
    ROS_ERROR("Failed to call (real) magnetize gripper service");
  }
  if (!open_gripper_client.call(trigger)) {
    ROS_ERROR("Failed to call (real) open gripper service");
  }
  mbzirc_comm_objs::DetectTypes set_detection_srv;
  set_detection_srv.request.types.push_back(_goal->color);
  set_detection_srv.request.command = mbzirc_comm_objs::DetectTypes::Request::COMMAND_TRACK_BRICK;
  set_detection_srv.request.visualize = false;
  if (!set_detection_client.call(set_detection_srv)) {
    ROS_ERROR("Failed to call set detection service");
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
      break;
    }

    const float height_threshold = 1.0;
    geometry_msgs::PoseStamped white_edge_pose;
    white_edge_pose.header = matched_candidate_.header;
    white_edge_pose.pose.position = matched_candidate_.point_of_interest;
    white_edge_pose.pose.orientation.z = 1;

    geometry_msgs::PoseStamped error_pose;
    tf2::doTransform(white_edge_pose, error_pose, optical_to_camera_control);
    error_pose.header.stamp = ros::Time::now();
    if (matched_candidate_.is_cropped || (sf11_range_.range < height_threshold)) {  // TODO: Threshold
      auto current_pose = ual_->pose();
      double absolute_yaw = 2.0 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);
      double absolute_yaw_error = normalizeAngle(absolute_yaw_lock - absolute_yaw);
      error_pose.pose.orientation.x = 0;
      error_pose.pose.orientation.y = 0;
      error_pose.pose.orientation.z = sin(0.5 * absolute_yaw_error);
      error_pose.pose.orientation.w = cos(0.5 * absolute_yaw_error);
    } else {
      error_pose.pose.orientation = matched_candidate_.pose.pose.orientation;
      error_pose.pose.orientation.z = -error_pose.pose.orientation.z;  // change sign!
      auto current_pose = ual_->pose();
      absolute_yaw_lock = 2.0 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    }

    geometry_msgs::Point error_position = error_pose.pose.position;
    if (since_last_candidate < timeout) {
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
      sleep(0.4);  // TODO: sleep?
      auto up_pose = ual_->pose();
      up_pose.pose.position.z += 2.0;  // TODO: Up?
      ual_->setPose(up_pose);
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
  set_detection_srv.request.command = mbzirc_comm_objs::DetectTypes::Request::COMMAND_STOP_TRACKING;
  if (!set_detection_client.call(set_detection_srv)) {
    ROS_ERROR("Failed to call set detection service");
  }
}

void UalActionServer::placeCallback(const mbzirc_comm_objs::PlaceGoalConstPtr &_goal) {
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