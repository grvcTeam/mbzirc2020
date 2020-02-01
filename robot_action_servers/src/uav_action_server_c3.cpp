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
#include <fire_extinguisher/fire_data.h>
#include <scan_passage_detection/wall_utils.h>

void UalActionServer::wallListCallback(const mbzirc_comm_objs::WallListConstPtr& msg) {
  wall_list_ = *msg;
}

void UalActionServer::moveInCirclesCallback(const mbzirc_comm_objs::MoveInCirclesGoalConstPtr &_goal) {
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

void UalActionServer::extinguishFacadeFireCallback(const mbzirc_comm_objs::ExtinguishFacadeFireGoalConstPtr &_goal) {
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

  ros::Rate loop_rate(FACADE_EXTINGUISH_LOOP_RATE);
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
    // velocity.twist.linear.x = x_pid.control_signal(-x_error, 1.0/FACADE_EXTINGUISH_LOOP_RATE);
    // velocity.twist.linear.y = y_pid.control_signal(-y_error, 1.0/FACADE_EXTINGUISH_LOOP_RATE);
    // velocity.twist.linear.z = z_pid.control_signal(z_error, 1.0/FACADE_EXTINGUISH_LOOP_RATE);
    // velocity.twist.angular.z = yaw_pid.control_signal(yaw_error, 1.0/FACADE_EXTINGUISH_LOOP_RATE);
    geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);

    // std::cout << velocity << '\n';
    ual_->setVelocity(velocity);

    loop_rate.sleep();
  }
}

void UalActionServer::extinguishGroundFireCallback(const mbzirc_comm_objs::ExtinguishGroundFireGoalConstPtr &_goal) {
  mbzirc_comm_objs::ExtinguishGroundFireFeedback feedback;
  mbzirc_comm_objs::ExtinguishGroundFireResult result;

  if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
    result.message = "UAL is not flying auto!";
    extinguish_ground_fire_server_.setAborted(result);
    return;
  }

  geometry_msgs::TransformStamped camera_to_gripper;  // Constant!
  try {
    camera_to_gripper = tf_buffer_.lookupTransform(tf_prefix_ + "/gripper_link", tf_prefix_ + "/camera_color_optical_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  ros::NodeHandle nh;
  ros::Subscriber sensed_sub = nh.subscribe("tracked_object", 1, &UalActionServer::trackedFireCallback, this);
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
  ros::ServiceClient release_blanket_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/release_blanket");
  ros::ServiceClient set_detection_client = nh.serviceClient<mbzirc_comm_objs::DetectTypes>("set_types");
  ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?
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
  pid_x.kp = 0.82;
  pid_x.ki = 0.0;
  pid_x.kd = 0.0;
  pid_x.min_sat = -0.5;
  pid_x.max_sat = 0.5;
  pid_x.min_wind = -0.5;
  pid_x.max_wind = 0.5;

  grvc::ual::PIDParams pid_y;
  pid_y.kp = 0.82;
  pid_y.ki = 0.0;
  pid_y.kd = 0.0;
  pid_y.min_sat = -0.5;
  pid_y.max_sat = 0.5;
  pid_y.min_wind = -0.5;
  pid_y.max_wind = 0.5;

  grvc::ual::PIDParams pid_z;
  pid_z.kp = 0.5;
  pid_z.ki = 0.0;
  pid_z.kd = 0.0;
  pid_z.min_sat = -0.5;
  pid_z.max_sat = 0.5;
  pid_z.min_wind = -0.5;
  pid_z.max_wind = 0.5;

  grvc::ual::PIDParams pid_yaw;
  pid_yaw.kp = 0.4;
  pid_yaw.ki = 0.02;
  pid_yaw.kd = 0.0;
  pid_yaw.min_sat = -0.5;
  pid_yaw.max_sat = 0.5;
  pid_yaw.min_wind = -0.5;
  pid_yaw.max_wind = 0.5;
  pid_yaw.is_angular = true;

  grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
  pose_pid.enableRosInterface("extinguish_ground_control");

  mbzirc_comm_objs::DetectTypes set_detection_srv;
  set_detection_srv.request.types.push_back("fire");
  set_detection_srv.request.command = mbzirc_comm_objs::DetectTypes::Request::COMMAND_TRACK_FIRE;
  set_detection_srv.request.visualize = true;
  if (!set_detection_client.call(set_detection_srv)) {
    ROS_ERROR("Failed to call set detection service");
  }

  grvc::utils::CircularBuffer history_xy_errors, history_z_errors;
  history_xy_errors.set_size(AVG_XY_ERROR_WINDOW_SIZE);
  history_xy_errors.fill_with(MAX_AVG_XY_ERROR);
  history_z_errors.set_size(AVG_Z_ERROR_WINDOW_SIZE);
  history_z_errors.fill_with(MAX_AVG_Z_ERROR);
  ros::Duration timeout(CANDIDATE_TIMEOUT);
  ros::Rate loop_rate(GROUND_EXTINGUISH_LOOP_RATE);
  while (ros::ok()) {
    if (extinguish_ground_fire_server_.isPreemptRequested()) {
      ual_->setPose(ual_->pose());
      extinguish_ground_fire_server_.setPreempted();
      break;
    }
    ros::Duration since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
    // ROS_INFO("since_last_candidate = %lf, timeout = %lf", since_last_candidate.toSec(), timeout.toSec());

    const float release_height = 1.5;
    geometry_msgs::PoseStamped fire_pose;
    fire_pose.header = matched_candidate_.header;
    fire_pose.pose = matched_candidate_.pose.pose;
    geometry_msgs::PoseStamped error_pose;
    tf2::doTransform(fire_pose, error_pose, camera_to_gripper);
    error_pose.header.stamp = ros::Time::now();
    //error_pose.header.frame_id = tf_prefix_ + "/gripper_link";
    // TODO: Get fire orientation
    error_pose.pose.orientation.x = 0.0;
    error_pose.pose.orientation.y = 0.0;
    error_pose.pose.orientation.z = 0.0;
    error_pose.pose.orientation.w = 1.0;

    geometry_msgs::Point error_position = error_pose.pose.position;

    if (since_last_candidate < timeout) {
      // x-y-control: in candidateCallback
      // z-control: descend until release_height
      double xy_error = sqrt(error_position.x*error_position.x + error_position.y*error_position.y);  // TODO: sqrt?
      history_xy_errors.push(xy_error);
      double min_xy_error, avg_xy_error, max_xy_error;
      history_xy_errors.get_stats(min_xy_error, avg_xy_error, max_xy_error);
      double z_error = release_height - sf11_range_.range;
      history_z_errors.push(z_error);
      double min_z_error, avg_z_error, max_z_error;
      history_z_errors.get_stats(min_z_error, avg_z_error, max_z_error);

      if (avg_xy_error > MAX_AVG_XY_ERROR) {
        avg_xy_error = MAX_AVG_XY_ERROR;
      }

      // Check if we are in position to release
      if (fabs(avg_z_error) < RELEASE_Z_ERROR_THRESHOLD && fabs(avg_xy_error) < RELEASE_XY_ERROR_THRESHOLD) {
        std_srvs::Trigger trigger;
        release_blanket_client.call(trigger);
        ual_->setPose(ual_->pose());
        extinguish_ground_fire_server_.setSucceeded(result);
        break;
      }
      error_pose.pose.position.z = z_error * (1.0 - (avg_xy_error / MAX_AVG_XY_ERROR));
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

    geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);
    ual_->setVelocity(velocity);

    loop_rate.sleep();
  }

  set_detection_srv.request.command = mbzirc_comm_objs::DetectTypes::Request::COMMAND_STOP_TRACKING;
  set_detection_srv.request.visualize = true;
  if (!set_detection_client.call(set_detection_srv)) {
    ROS_ERROR("Failed to call set detection service");
  }
}