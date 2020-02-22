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
#include <scan_passage_detection/wall_utils.h>
#include <mbzirc_comm_objs/CheckFire.h>

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

geometry_msgs::PoseStamped UalActionServer::errorPoseFromFireData(const FireData& _target_fire, bool _publish_markers) {
  // TODO: target criteria
  mbzirc_comm_objs::Wall target_wall = closestWall(_target_fire.wall_list);
  // mbzirc_comm_objs::Wall target_wall = largestWall(_target_fire.wall_list);
  mbzirc_comm_objs::Wall current_wall = closestWall(wall_list_);
  // mbzirc_comm_objs::Wall current_wall = largestWall(wall_list_);

  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA color;
  if (_publish_markers) {
    color.r = 0.0;  // Color for target_wall
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;
    marker_array.markers.push_back(getLineMarker(target_wall, wall_list_.header.frame_id, color, 0, "target_wall"));
    color.r = 0.0;  // Prepare color for healthy current wall...
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;
  }

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
      // ROS_INFO("xy: %lf, %lf", x_error, y_error);
  } else {
      ROS_WARN("Wall lengths too different! squared_delta_length = %lf", squared_delta_length);
      // std::cout << target_wall;
      // std::cout << current_wall;
      color.r = 1.0;  // ...but current wall is not healthy :(
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
  }

  if (_publish_markers) {
    marker_array.markers.push_back(getLineMarker(current_wall, wall_list_.header.frame_id, color, 1, "target_wall"));
    marker_pub_.publish(marker_array);
  }

  // ROS_ERROR("start: %lf, %lf", x_start_error, y_start_error);
  // ROS_ERROR("end: %lf, %lf", x_end_error, y_end_error);
  // ROS_ERROR("xy: %lf, %lf", x_error, y_error);
  // std::cout << current_wall;

  double z_error = _target_fire.sf11_range.range - sf11_range_.range;
  // ROS_ERROR("z: %lf", z_error);

  auto ual_pose = ual_->pose();
  double current_yaw = yawFromPose(ual_pose.pose);
  double fire_ual_yaw = yawFromPose(_target_fire.ual_pose.pose);
  double yaw_error = fire_ual_yaw - current_yaw;

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
  return error_pose;
}

void UalActionServer::extinguishFacadeFireCallback(const mbzirc_comm_objs::ExtinguishFacadeFireGoalConstPtr &_goal) {
  mbzirc_comm_objs::ExtinguishFacadeFireFeedback feedback;
  mbzirc_comm_objs::ExtinguishFacadeFireResult result;

  if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
    result.message = "UAL is not flying auto!";
    extinguish_facade_fire_server_.setAborted(result);
    return;
  }
/*
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
*/

  geometry_msgs::TransformStamped optical_to_camera;  // Constant!
  try {
    optical_to_camera = tf_buffer_.lookupTransform(tf_prefix_ + "/camera_link", tf_prefix_ + "/camera_color_optical_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    result.message = ex.what();
    extinguish_facade_fire_server_.setAborted(result);
  }

  ros::NodeHandle nh;
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
  ros::Subscriber walls_sub = nh.subscribe<mbzirc_comm_objs::WallList>("walls", 1, &UalActionServer::wallListCallback, this);
  ros::Subscriber sensed_sub = nh.subscribe<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);
  ros::ServiceClient start_pump_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/start_pump");
  ros::ServiceClient stop_pump_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/stop_pump");
  ros::ServiceClient check_fire_client = nh.serviceClient<mbzirc_comm_objs::CheckFire>("thermal_detection/fire_detected");
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
  ros::Publisher debug_pub = nh.advertise<geometry_msgs::TwistStamped>("/debug_velocity", 1);

  if (!waitForFreshSf11RangeMsg(10)) {
    result.message = "could not get fresh [sf11_range]";
    extinguish_facade_fire_server_.setAborted(result);
    return;
  }

  if (!waitForFreshWallListMsg(10)) {
    result.message = "could not get fresh [wall_list]";
    extinguish_facade_fire_server_.setAborted(result);
    return;
  }

  grvc::ual::PIDParams pid_x;
  pid_x.kp = 0.4;
  pid_x.ki = 0.0;
  pid_x.kd = 0.0;
  pid_x.min_sat = -0.5;
  pid_x.max_sat = 0.5;
  pid_x.min_wind = -0.1;
  pid_x.max_wind = 0.1;

  grvc::ual::PIDParams pid_y;
  pid_y.kp = 0.5;
  pid_y.ki = 0.0;
  pid_y.kd = 0.0;
  pid_y.min_sat = -0.5;
  pid_y.max_sat = 0.5;
  pid_y.min_wind = -0.1;
  pid_y.max_wind = 0.1;

  grvc::ual::PIDParams pid_z;
  pid_z.kp = 0.4;
  pid_z.ki = 0.0;
  pid_z.kd = 0.0;
  pid_z.min_sat = -0.5;
  pid_z.max_sat = 0.5;
  pid_z.min_wind = -0.1;
  pid_z.max_wind = 0.1;

  grvc::ual::PIDParams pid_yaw;
  pid_yaw.kp = 0.4;
  pid_yaw.ki = 0.02;
  pid_yaw.kd = 0.0;
  pid_yaw.min_sat = -0.5;
  pid_yaw.max_sat = 0.5;
  pid_yaw.min_wind = -0.1;
  pid_yaw.max_wind = 0.1;
  pid_yaw.is_angular = true;

  grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
  // pose_pid.enableRosInterface("extinguish_facade_control");

  grvc::utils::CircularBuffer history_sq_xyz_errors;
  history_sq_xyz_errors.set_size(FACADE_EXTINGUISH_LOOP_RATE);  // TODO: 1s
  history_sq_xyz_errors.fill_with(1e3);  // TODO: 1km?

  FireData target_fire;
  bool fire_is_locked = false;
  auto ual_safe_pose = ual_->pose();
  mbzirc_comm_objs::ObjectDetection hole;
  ros::Duration timeout(CANDIDATE_TIMEOUT);  // TODO: different timeout?
  ros::Rate loop_rate(FACADE_EXTINGUISH_LOOP_RATE);
  while (ros::ok()) {
    if (extinguish_facade_fire_server_.isPreemptRequested()) {
      ual_->setPose(ual_->pose());
      extinguish_facade_fire_server_.setPreempted();
      return;
    }

    float min_sq_distance = 1e6;
    for (size_t i = 0; i < sensed_objects_.objects.size(); i++) {
      auto object = sensed_objects_.objects[i];
      if (object.type == mbzirc_comm_objs::ObjectDetection::TYPE_HOLE) {
        auto current_sq_distance = squaredPositionNorm(object.pose.pose);
        if (current_sq_distance < min_sq_distance) {
          min_sq_distance = current_sq_distance;
          hole = object;  // TODO: Closest is best fit?
        }
      }
    }
    if (min_sq_distance > 36) {  // TODO: Tune?
      if (min_sq_distance < 1e5) {
        ROS_WARN("Closest hole is too far!");
      }
      continue;
    }
    ros::Duration since_last_candidate = ros::Time::now() - hole.header.stamp;

    if (since_last_candidate < timeout) {
      mbzirc_comm_objs::Wall closest_wall = closestWall(wall_list_);
      if ((closest_wall.start[0] == 0)) {
        ROS_WARN("No closest wall!");
       continue;
      }

      double distance_to_closest_wall = 0;
      if ((closest_wall.start[0] != closest_wall.end[0]) || (closest_wall.start[1] != closest_wall.end[1])) {
        distance_to_closest_wall = squaredDistanceToSegment(0, 0, closest_wall.start[0], closest_wall.start[1], closest_wall.end[0], closest_wall.end[1]);
      }
      if (distance_to_closest_wall == 0) {
        continue;
      }

      if (distance_to_closest_wall < 2.25) {  // TODO: Tune (2.25 = 1.5*1.5)
        ROS_WARN("Too close to closest_wall!");
        ual_->setPose(ual_safe_pose);
      } else {
        ual_safe_pose = ual_->pose();

        geometry_msgs::PoseStamped hole_pose;
        hole_pose.header = hole.header;
        hole_pose.pose = hole.pose.pose;
        // std::cout << hole_pose << '\n';
        geometry_msgs::PoseStamped error_pose;
        tf2::doTransform(hole_pose, error_pose, optical_to_camera);
        error_pose.header.stamp = ros::Time::now();
        // std::cout << error_pose << '\n';
        error_pose.pose.position.x -= 1.80;  // TODO!
        error_pose.pose.position.y += 0.00;  // TODO!
        error_pose.pose.position.z += 0.17;  // TODO!

        // Control yaw with closest wall
        float dx, dy;
        if (closest_wall.start[1] < closest_wall.end[1]) {
          dx = closest_wall.end[0] - closest_wall.start[0];
          dy = closest_wall.end[1] - closest_wall.start[1];
        } else {
          dx = closest_wall.start[0] - closest_wall.end[0];
          dy = closest_wall.start[1] - closest_wall.end[1];
        }
        if ((dx == 0) && (dy == 0)) {
          ROS_WARN("dx = dy = 0");
          continue;
        }
        // ROS_INFO("dx = %f, dy = %f", dx, dy);
        float wall_angle = atan2(dx, dy);  // Defined in this way on purpose! (usually dy/dx)
        // ROS_INFO("wall_angle = %f", wall_angle);
        error_pose.pose.orientation.x = 0;
        error_pose.pose.orientation.y = 0;
        error_pose.pose.orientation.z = -sin(0.5 * wall_angle);
        error_pose.pose.orientation.w =  cos(0.5 * wall_angle);
        // std::cout << error_pose << '\n';
        geometry_msgs::TwistStamped velocity;
        velocity = pose_pid.updateError(error_pose);
        // velocity.twist.angular.x = 0;  // TODO!
        // velocity.twist.angular.y = 0;  // TODO!
        // velocity.twist.angular.z = 0;  // TODO!
        // std::cout << velocity << '\n';
        // debug_pub.publish(velocity);
        ual_->setVelocity(velocity);

        history_sq_xyz_errors.push(squaredPositionNorm(error_pose.pose));
        double min_sq_xyz_error, avg_sq_xyz_error, max_sq_xyz_error;
        history_sq_xyz_errors.get_stats(min_sq_xyz_error, avg_sq_xyz_error, max_sq_xyz_error);

        if ((avg_sq_xyz_error < 0.01) && (fabs(yawFromPose(error_pose.pose)) < 0.1)) {  // TODO: tune!
          auto current_pose = ual_->pose();
          target_fire.wall_list = wall_list_;
          target_fire.sf11_range = sf11_range_;
          target_fire.id = "TODO";  // TODO!
          target_fire.ual_pose = current_pose;
          ual_->goToWaypoint(current_pose, false);  // TODO: is this enough?
          fire_is_locked = true;
          ROS_INFO("Fire locked!");
          break;
        }
      }

    } else {
      ROS_WARN("Candidates timeout!");
      ual_->setPose(ual_safe_pose);
    }

    loop_rate.sleep();
  }

  if (!fire_is_locked) {
    ual_->setPose(ual_->pose());
    result.message = "Could not lock fire";
    extinguish_facade_fire_server_.setAborted(result);
    return;
  }

  ros::Duration(3).sleep();
  mbzirc_comm_objs::CheckFire check_fire_srv;
  check_fire_client.call(check_fire_srv);
  bool fire_detected = check_fire_srv.response.fire_detected;
  // bool fire_detected = true;
  if (!fire_detected) {
    ual_->setPose(ual_->pose());
    result.message = "Fire is not detected";
    extinguish_facade_fire_server_.setAborted(result);
    return;
  }

  // TODO: Start pumping!
  std_srvs::Trigger trigger;
  start_pump_client.call(trigger);
//  ros::Duration(20).sleep();
//  stop_pump_client.call(trigger);
//  result.message = "Fire extinguished!";
//  extinguish_facade_fire_server_.setSucceeded(result);

  int pumping_count_loop = 0;
  while (ros::ok()) {
    mbzirc_comm_objs::Wall closest_wall = closestWall(wall_list_);
    double distance_to_closest_wall = 0.0;
    if ((closest_wall.start[0] != closest_wall.end[0]) || (closest_wall.start[1] != closest_wall.end[1])) {
      distance_to_closest_wall = squaredDistanceToSegment(0, 0, closest_wall.start[0], closest_wall.start[1], closest_wall.end[0], closest_wall.end[1]);
    }

    if (distance_to_closest_wall < 2.25) {  // TODO: Tune (1.5*1.5 = 2.25)
      ROS_WARN("Too close to closest_wall!");
      ual_->setPose(ual_safe_pose);
    } else {
      ual_safe_pose = ual_->pose();
      auto error_pose = errorPoseFromFireData(target_fire, true);
      if (squaredPositionNorm(error_pose.pose) > 1.0) {
        error_pose.pose.position.x = 0;
        error_pose.pose.position.y = 0;
        // Do not reset z nor orientation on purpose
      }
      geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);
      ual_->setVelocity(velocity);
    }

    if ((pumping_count_loop / FACADE_EXTINGUISH_LOOP_RATE) > 20) {  // TODO: Tune, in seconds
      stop_pump_client.call(trigger);
      ual_->setPose(ual_->pose());
      result.message = "Fire extinguished!";
      extinguish_facade_fire_server_.setSucceeded(result);
      break;
    }
    pumping_count_loop++;
    loop_rate.sleep();
  }

  // TODO: Save target_fire to file?
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
  ros::ServiceClient check_fire_client = nh.serviceClient<mbzirc_comm_objs::CheckFire>("thermal_detection/fire_detected");
  // TODO: Add other subscribers (e.g. fire estimation)
  // TODO: Markers?
  //ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

  if (!waitForFreshSf11RangeMsg(10)) {
    result.message = "could not get fresh [sf11_range]";
    extinguish_ground_fire_server_.setAborted(result);
    return;
  }

  grvc::ual::PIDParams pid_x;
  pid_x.kp = 0.5;
  pid_x.ki = 0.0;
  pid_x.kd = 0.0;
  pid_x.min_sat = -0.3;
  pid_x.max_sat = 0.3;
  pid_x.min_wind = -0.5;
  pid_x.max_wind = 0.5;

  grvc::ual::PIDParams pid_y;
  pid_y.kp = 0.5;
  pid_y.ki = 0.0;
  pid_y.kd = 0.0;
  pid_y.min_sat = -0.3;
  pid_y.max_sat = 0.3;
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
  pid_yaw.min_sat = -0.2;
  pid_yaw.max_sat = 0.2;
  pid_yaw.min_wind = -0.5;
  pid_yaw.max_wind = 0.5;
  pid_yaw.is_angular = true;

  grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
  pose_pid.enableRosInterface("extinguish_ground_control");

  // TODO: Where do we check this?
  // ros::Duration(3).sleep();
  // mbzirc_comm_objs::CheckFire check_fire_srv;
  // check_fire_client.call(check_fire_srv);
  // bool fire_detected = check_fire_srv.response.fire_detected;
  // // bool fire_detected = true;
  // if (!fire_detected) {
  //   ual_->setPose(ual_->pose());
  //   result.message = "Fire is not detected";
  //   extinguish_ground_fire_server_.setAborted(result);
  //   return;
  // }

  std::string fire_color = _goal->color;
  if ((fire_color != "fire") && (fire_color != "fire_box")) {
    ROS_WARN("Unexpected fire color [%s], setting it to fire", fire_color.c_str());
    fire_color = "fire";
  }

  mbzirc_comm_objs::DetectTypes set_detection_srv;
  set_detection_srv.request.types.push_back(fire_color);
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

    const float release_height = 2.5;
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
        auto up_pose = ual_->pose();
        up_pose.pose.position.z += 3.0;  // TODO: Value
        ual_->setPose(up_pose);
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

void UalActionServer::lookForGroundFiresCallback(const mbzirc_comm_objs::LookForGroundFiresGoalConstPtr &_goal) {
  mbzirc_comm_objs::LookForGroundFiresFeedback feedback;
  mbzirc_comm_objs::LookForGroundFiresResult result;

  if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
    result.message = "UAL is not flying auto!";
    look_for_ground_fires_server_.setAborted(result);
    return;
  }

  ros::NodeHandle nh;
  ros::ServiceClient set_detection_client = nh.serviceClient<mbzirc_comm_objs::DetectTypes>("set_types");
  ros::ServiceClient check_fire_client = nh.serviceClient<mbzirc_comm_objs::CheckFire>("thermal_detection/fire_detected");
  ros::Subscriber sensed_sub = nh.subscribe<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);

  mbzirc_comm_objs::DetectTypes set_detection_srv;
  set_detection_srv.request.command = mbzirc_comm_objs::DetectTypes::Request::COMMAND_DETECT_ALL;
  set_detection_srv.request.visualize = true;
  if (!set_detection_client.call(set_detection_srv)) {
    ROS_ERROR("Failed to call set detection service");
  }

  bool fire_found = false;
  bool fire_detected = false;
  int waypoint_id = 0;
  std::vector<geometry_msgs::PoseStamped> path = _goal->path;
  if(path.size()==0) {
    result.message = "Empty path given!";
    look_for_ground_fires_server_.setAborted(result);
    return;
  }
  ros::Rate loop_rate(GROUND_EXTINGUISH_LOOP_RATE);

  while(!fire_detected && ros::ok()) {
    if (look_for_ground_fires_server_.isPreemptRequested()) {
      ual_->setPose(ual_->pose());
      look_for_ground_fires_server_.setPreempted();
      break;
    }

    for (size_t i = 0; i < sensed_objects_.objects.size(); i++) {
      auto object = sensed_objects_.objects[i];
      if (object.color == mbzirc_comm_objs::ObjectDetection::COLOR_FIRE) {
        fire_found = true;
        }
      }

    if(fire_found) {
      mbzirc_comm_objs::CheckFire check_fire_srv;
      check_fire_client.call(check_fire_srv);
      fire_detected = check_fire_srv.response.fire_detected;
      if(fire_detected) {
        ual_->setPose(ual_->pose());
        result.message = "Fire detected!";
        look_for_ground_fires_server_.setSucceeded(result);
        break;
      }
    }

    if(ual_->isIdle() && !fire_detected) {
      waypoint_id++;
      if(waypoint_id < path.size()) {
        ual_->goToWaypoint(path[waypoint_id], false);
        ros::Duration(0.5).sleep();
      }
      else {
        result.message = "Path finalized without finding fires!";
        look_for_ground_fires_server_.setAborted(result);
        break;
      }
    }

    loop_rate.sleep();
  }
}

void UalActionServer::goToFacadeFireCallback(const mbzirc_comm_objs::GoToFacadeFireGoalConstPtr &_goal) {}
/*
  mbzirc_comm_objs::GoToFacadeFireFeedback feedback;
  mbzirc_comm_objs::GoToFacadeFireResult result;

  if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO) {
    result.message = "UAL is not flying auto!";
    go_to_facade_fire_server_.setAborted(result);
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
    go_to_facade_fire_server_.setAborted(result);
    return;
  }
  FireData target_fire = fire_map[_goal->fire_id];
/*
  geometry_msgs::TransformStamped optical_to_camera;  // Constant!
  try {
    optical_to_camera = tf_buffer_.lookupTransform(tf_prefix_ + "/camera_link", tf_prefix_ + "/camera_color_optical_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    result.message = ex.what();
    go_to_facade_fire_server_.setAborted(result);
  }
*/
/*
  ros::NodeHandle nh;
  ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("sf11", 1, &UalActionServer::sf11RangeCallback, this);
  ros::Subscriber walls_sub = nh.subscribe<mbzirc_comm_objs::WallList>("walls", 1, &UalActionServer::wallListCallback, this);
  // ros::Subscriber sensed_sub = nh.subscribe<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);
  // ros::ServiceClient start_pump_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/start_pump");
  // ros::ServiceClient stop_pump_client = nh.serviceClient<std_srvs::Trigger>("actuators_system/stop_pump");
  // ros::ServiceClient check_fire_client = nh.serviceClient<mbzirc_comm_objs::CheckFire>("thermal_detection/fire_detected");
  // ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
  ros::Publisher debug_pub = nh.advertise<geometry_msgs::TwistStamped>("/debug_velocity", 1);

  if (!waitForFreshSf11RangeMsg(10)) {
    result.message = "could not get fresh [sf11_range]";
    go_to_facade_fire_server_.setAborted(result);
    return;
  }

  if (!waitForFreshWallListMsg(10)) {
    result.message = "could not get fresh [wall_list]";
    go_to_facade_fire_server_.setAborted(result);
    return;
  }

  grvc::ual::PIDParams pid_x;
  pid_x.kp = 0.4;
  pid_x.ki = 0.0;
  pid_x.kd = 0.0;
  pid_x.min_sat = -0.5;
  pid_x.max_sat = 0.5;
  pid_x.min_wind = -0.1;
  pid_x.max_wind = 0.1;

  grvc::ual::PIDParams pid_y;
  pid_y.kp = 0.5;
  pid_y.ki = 0.0;
  pid_y.kd = 0.0;
  pid_y.min_sat = -0.5;
  pid_y.max_sat = 0.5;
  pid_y.min_wind = -0.1;
  pid_y.max_wind = 0.1;

  grvc::ual::PIDParams pid_z;
  pid_z.kp = 0.4;
  pid_z.ki = 0.0;
  pid_z.kd = 0.0;
  pid_z.min_sat = -0.5;
  pid_z.max_sat = 0.5;
  pid_z.min_wind = -0.1;
  pid_z.max_wind = 0.1;

  grvc::ual::PIDParams pid_yaw;
  pid_yaw.kp = 0.4;
  pid_yaw.ki = 0.02;
  pid_yaw.kd = 0.0;
  pid_yaw.min_sat = -0.5;
  pid_yaw.max_sat = 0.5;
  pid_yaw.min_wind = -0.1;
  pid_yaw.max_wind = 0.1;
  pid_yaw.is_angular = true;

  grvc::ual::PosePID pose_pid(pid_x, pid_y, pid_z, pid_yaw);
  // pose_pid.enableRosInterface("go_to_facade_control");

  grvc::utils::CircularBuffer history_sq_xyz_errors;
  history_sq_xyz_errors.set_size(FACADE_EXTINGUISH_LOOP_RATE);  // TODO: 1s
  history_sq_xyz_errors.fill_with(1e3);  // TODO: 1km?

/*
  FireData target_fire;
  bool fire_is_locked = false;
  auto ual_safe_pose = ual_->pose();
  mbzirc_comm_objs::ObjectDetection hole;
  ros::Duration timeout(CANDIDATE_TIMEOUT);  // TODO: different timeout?
  ros::Rate loop_rate(FACADE_EXTINGUISH_LOOP_RATE);
  while (ros::ok()) {
    if (go_to_facade_fire_server_.isPreemptRequested()) {
      ual_->setPose(ual_->pose());
      go_to_facade_fire_server_.setPreempted();
      return;
    }

    float min_sq_distance = 1e6;
    for (size_t i = 0; i < sensed_objects_.objects.size(); i++) {
      auto object = sensed_objects_.objects[i];
      if (object.type == mbzirc_comm_objs::ObjectDetection::TYPE_HOLE) {
        auto current_sq_distance = squaredPositionNorm(object.pose.pose);
        if (current_sq_distance < min_sq_distance) {
          min_sq_distance = current_sq_distance;
          hole = object;  // TODO: Closest is best fit?
        }
      }
    }
    if (min_sq_distance > 36) {  // TODO: Tune?
      if (min_sq_distance < 1e5) {
        ROS_WARN("Closest hole is too far!");
      }
      continue;
    }
    ros::Duration since_last_candidate = ros::Time::now() - hole.header.stamp;

    if (since_last_candidate < timeout) {
      mbzirc_comm_objs::Wall closest_wall = closestWall(wall_list_);
      if ((closest_wall.start[0] == 0)) {
        ROS_WARN("No closest wall!");
       continue;
      }
*/
/*
    auto ual_safe_pose = ual_->pose();
    ual_->goToWaypoint(_goal->waypoint, false);  // TODO: timeout?
      while(!ual_->isIdle() && ros::ok()) {
        if (go_to_server_.isPreemptRequested()) {
          ual_->setPose(ual_->pose());
          go_to_server_.setPreempted();
          return;
        }
        loop_rate.sleep();
      }
      go_to_server_.setSucceeded(result);

    while (ros::ok()) {
      double distance_to_closest_wall = 0;
      mbzirc_comm_objs::Wall closest_wall = closestWall(wall_list_);
      if ((closest_wall.start[0] != closest_wall.end[0]) || (closest_wall.start[1] != closest_wall.end[1])) {
        distance_to_closest_wall = squaredDistanceToSegment(0, 0, closest_wall.start[0], closest_wall.start[1], closest_wall.end[0], closest_wall.end[1]);
      }
      if (distance_to_closest_wall == 0) {
        continue;
      }

      if (distance_to_closest_wall < 2.25) {  // TODO: Tune (2.25 = 1.5*1.5)
        ROS_WARN("Too close to closest_wall!");
        ual_->setPose(ual_safe_pose);
      } else {
        ual_safe_pose = ual_->pose();

        // geometry_msgs::PoseStamped hole_pose;
        // hole_pose.header = hole.header;
        // hole_pose.pose = hole.pose.pose;
        // // std::cout << hole_pose << '\n';
        // geometry_msgs::PoseStamped error_pose;
        // tf2::doTransform(hole_pose, error_pose, optical_to_camera);
        // error_pose.header.stamp = ros::Time::now();
        // // std::cout << error_pose << '\n';
        // error_pose.pose.position.x -= 1.80;  // TODO!
        // error_pose.pose.position.y += 0.00;  // TODO!
        // error_pose.pose.position.z += 0.17;  // TODO!

        // Control yaw with closest wall
        float dx, dy;
        if (closest_wall.start[1] < closest_wall.end[1]) {
          dx = closest_wall.end[0] - closest_wall.start[0];
          dy = closest_wall.end[1] - closest_wall.start[1];
        } else {
          dx = closest_wall.start[0] - closest_wall.end[0];
          dy = closest_wall.start[1] - closest_wall.end[1];
        }
        if ((dx == 0) && (dy == 0)) {
          ROS_WARN("dx = dy = 0");
          continue;
        }
        // ROS_INFO("dx = %f, dy = %f", dx, dy);
        float wall_angle = atan2(dx, dy);  // Defined in this way on purpose! (usually dy/dx)
        // ROS_INFO("wall_angle = %f", wall_angle);
        error_pose.pose.orientation.x = 0;
        error_pose.pose.orientation.y = 0;
        error_pose.pose.orientation.z = -sin(0.5 * wall_angle);
        error_pose.pose.orientation.w =  cos(0.5 * wall_angle);
        // std::cout << error_pose << '\n';
        geometry_msgs::TwistStamped velocity;
        velocity = pose_pid.updateError(error_pose);
        // velocity.twist.angular.x = 0;  // TODO!
        // velocity.twist.angular.y = 0;  // TODO!
        // velocity.twist.angular.z = 0;  // TODO!
        // std::cout << velocity << '\n';
        // debug_pub.publish(velocity);
        ual_->setVelocity(velocity);

        history_sq_xyz_errors.push(squaredPositionNorm(error_pose.pose));
        double min_sq_xyz_error, avg_sq_xyz_error, max_sq_xyz_error;
        history_sq_xyz_errors.get_stats(min_sq_xyz_error, avg_sq_xyz_error, max_sq_xyz_error);

        if ((avg_sq_xyz_error < 0.01) && (fabs(yawFromPose(error_pose.pose)) < 0.1)) {  // TODO: tune!
          auto current_pose = ual_->pose();
          target_fire.wall_list = wall_list_;
          target_fire.sf11_range = sf11_range_;
          target_fire.id = "TODO";  // TODO!
          target_fire.ual_pose = current_pose;
          ual_->goToWaypoint(current_pose, false);  // TODO: is this enough?
          fire_is_locked = true;
          ROS_INFO("Fire locked!");
          break;
        }
      }

    } else {
      ROS_WARN("Candidates timeout!");
      ual_->setPose(ual_safe_pose);
    }

    loop_rate.sleep();
  }

  if (!fire_is_locked) {
    ual_->setPose(ual_->pose());
    result.message = "Could not lock fire";
    go_to_facade_fire_server_.setAborted(result);
    return;
  }

  ros::Duration(3).sleep();
  mbzirc_comm_objs::CheckFire check_fire_srv;
  check_fire_client.call(check_fire_srv);
  bool fire_detected = check_fire_srv.response.fire_detected;
  // bool fire_detected = true;
  if (!fire_detected) {
    ual_->setPose(ual_->pose());
    result.message = "Fire is not detected";
    go_to_facade_fire_server_.setAborted(result);
    return;
  }

  // TODO: Start pumping!
  // std_srvs::Trigger trigger;
  // start_pump_client.call(trigger);
//  ros::Duration(20).sleep();
//  stop_pump_client.call(trigger);
//  result.message = "Fire extinguished!";
//  go_to_facade_fire_server_.setSucceeded(result);

  // int pumping_count_loop = 0;
  auto ual_safe_pose = ual_->pose();
  while (ros::ok()) {
    mbzirc_comm_objs::Wall closest_wall = closestWall(wall_list_);
    double distance_to_closest_wall = 0.0;
    if ((closest_wall.start[0] != closest_wall.end[0]) || (closest_wall.start[1] != closest_wall.end[1])) {
      distance_to_closest_wall = squaredDistanceToSegment(0, 0, closest_wall.start[0], closest_wall.start[1], closest_wall.end[0], closest_wall.end[1]);
    }

    if (distance_to_closest_wall < 2.25) {  // TODO: Tune (1.5*1.5 = 2.25)
      ROS_WARN("Too close to closest_wall!");
      ual_->setPose(ual_safe_pose);
    } else {
      ual_safe_pose = ual_->pose();
      auto error_pose = errorPoseFromFireData(target_fire, true);
      if (squaredPositionNorm(error_pose.pose) > 1.0) {
        error_pose.pose.position.x = 0;
        error_pose.pose.position.y = 0;
        // Do not reset z nor orientation on purpose
      }
      geometry_msgs::TwistStamped velocity = pose_pid.updateError(error_pose);
      ual_->setVelocity(velocity);
    }

    if ((pumping_count_loop / FACADE_EXTINGUISH_LOOP_RATE) > 20) {  // TODO: Tune, in seconds
      stop_pump_client.call(trigger);
      ual_->setPose(ual_->pose());
      result.message = "Fire extinguished!";
      go_to_facade_fire_server_.setSucceeded(result);
      break;
    }
    pumping_count_loop++;
    loop_rate.sleep();
  }

  // TODO: Save target_fire to file?
}
*/