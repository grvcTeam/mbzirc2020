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
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/circular_buffer.h>
#include <handy_tools/pid_controller.h>
#include <math.h> /* atan2 */
#include <mbzirc_comm_objs/GoToAction.h>
#include <mbzirc_comm_objs/GripperAttached.h>
#include <mbzirc_comm_objs/LandAction.h>
#include <mbzirc_comm_objs/Magnetize.h>
#include <mbzirc_comm_objs/MoveInCirclesAction.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/PickAction.h>
#include <mbzirc_comm_objs/PlaceAction.h>
#include <mbzirc_comm_objs/RobotDataFeed.h>
#include <mbzirc_comm_objs/SearchTargetAction.h>
#include <mbzirc_comm_objs/TakeOffAction.h>
#include <mbzirc_comm_objs/TrackAction.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/ual.h>

class UalActionServer
{
  protected:
   ros::NodeHandle nh_;
   // NodeHandle instance must be created before this line. Otherwise strange error occurs
   actionlib::SimpleActionServer<mbzirc_comm_objs::TakeOffAction> take_off_server_;
   actionlib::SimpleActionServer<mbzirc_comm_objs::GoToAction> go_to_server_;
   actionlib::SimpleActionServer<mbzirc_comm_objs::SearchTargetAction> search_target_server_;
   actionlib::SimpleActionServer<mbzirc_comm_objs::TrackAction> track_server_;
   actionlib::SimpleActionServer<mbzirc_comm_objs::LandAction> land_server_;
   actionlib::SimpleActionServer<mbzirc_comm_objs::MoveInCirclesAction> move_in_circles_server_;

   tf2_ros::Buffer tf_buffer_;
   tf2_ros::TransformListener *tf_listener_;

   grvc::ual::UAL *ual_;
   ros::Timer data_feed_timer_;
   ros::Publisher data_feed_pub_;
   mbzirc_comm_objs::ObjectDetection matched_candidate_;
   geometry_msgs::PointStamped depth_detection_;

   bool object_sensed_       = false;
   bool object_sensed_depth_ = false;

  public:
   UalActionServer()
       : take_off_server_(nh_, "take_off_action", boost::bind(&UalActionServer::takeOffCallback, this, _1), false),
         go_to_server_(nh_, "go_to_action", boost::bind(&UalActionServer::goToCallback, this, _1), false),
         search_target_server_(nh_, "search_target_action",
                               boost::bind(&UalActionServer::searchTargetCallback, this, _1), false),
         track_server_(nh_, "track_action", boost::bind(&UalActionServer::trackCallback, this, _1), false),
         land_server_(nh_, "land_action", boost::bind(&UalActionServer::landCallback, this, _1), false),
         move_in_circles_server_(nh_, "move_in_circles_action",
                                 boost::bind(&UalActionServer::moveInCirclesCallback, this, _1), false)
   {
      tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

      ual_ = new grvc::ual::UAL();
      data_feed_timer_ =
          nh_.createTimer(ros::Duration(0.1), &UalActionServer::publishDataFeed, this);  // TODO: frequency?
      data_feed_pub_ = nh_.advertise<mbzirc_comm_objs::RobotDataFeed>("data_feed", 1);   // TODO: namespacing?
      // TODO: start servers only when needed?
      // TODO: make servers separated classes?
      take_off_server_.start();
      go_to_server_.start();
      search_target_server_.start();
      track_server_.start();
      land_server_.start();
      move_in_circles_server_.start();
   }

   ~UalActionServer() { delete ual_; }

   void publishDataFeed(const ros::TimerEvent &)
   {
      mbzirc_comm_objs::RobotDataFeed data_feed;
      data_feed.pose = ual_->pose();
      data_feed_pub_.publish(data_feed);
   }

   // TODO: Who should know about flight_level? robot_action_servers, uav_agent, central_agent...
   void takeOffCallback(const mbzirc_comm_objs::TakeOffGoalConstPtr &_goal)
   {
      // ROS_INFO("Take off!");
      // mbzirc_comm_objs::TakeOffFeedback feedback;
      mbzirc_comm_objs::TakeOffResult result;

      while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok())
      {
         ROS_WARN("UAL is uninitialized!");
         sleep(1);
      }
      // TODO: Fill result
      switch (ual_->state().state)
      {
         case uav_abstraction_layer::State::LANDED_DISARMED:
            ROS_WARN("UAL is disarmed!");
            take_off_server_.setAborted(result);
            break;
         case uav_abstraction_layer::State::LANDED_ARMED:
         {
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

   void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal)
   {
      // ROS_INFO("Go to!");
      // mbzirc_comm_objs::GoToFeedback feedback;
      mbzirc_comm_objs::GoToResult result;

      while ((ual_->state().state == uav_abstraction_layer::State::UNINITIALIZED) && ros::ok())
      {
         ROS_WARN("UAL is uninitialized!");
         sleep(1);
      }
      // TODO: Fill result
      switch (ual_->state().state)
      {
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

   void sensedObjectsCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr &msg)
   {
      float min_distance = 1e3;  // 1 Km
      for (auto object : msg->objects)
      {
         float distance = object.relative_position.x * object.relative_position.x +
                          object.relative_position.y * object.relative_position.y;
         if (distance < min_distance)
         {
            min_distance       = distance;
            matched_candidate_ = object;  // TODO: Check also color is correct
            object_sensed_     = true;
         }
      }
   }

   void depthDetectionCallback(const geometry_msgs::PointStampedConstPtr &msg)
   {
      depth_detection_     = *msg;
      object_sensed_depth_ = true;
   }

   void searchTargetCallback(const mbzirc_comm_objs::SearchTargetGoalConstPtr &_goal)
   {
      // ROS_INFO("Pick!");
      // mbzirc_comm_objs::PickFeedback feedback;
      mbzirc_comm_objs::SearchTargetResult result;

      if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO)
      {
         ROS_WARN("UAL is not flying auto!");
         search_target_server_.setAborted(result);
         return;
      }

      ros::NodeHandle nh;
      ros::Subscriber sensed_sub_ =
          nh.subscribe("color_detector/sensed_objects", 1, &UalActionServer::sensedObjectsCallback, this);

      ros::Subscriber depth_sensed_sub_ =
          nh.subscribe("drone_detector/detection_points", 1, &UalActionServer::depthDetectionCallback, this);

      ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?

      if (_goal->lower_bound.x > _goal->upper_bound.x || _goal->lower_bound.y > _goal->upper_bound.y ||
          _goal->lower_bound.z > _goal->upper_bound.z)
      {
         ROS_WARN("Invalid goal lower bound is higher than upper bound!");
         search_target_server_.setAborted(result);
         return;
      }

      double search_size_length = _goal->upper_bound.y - _goal->lower_bound.y;

      uint i        = 5;
      double offset = search_size_length / (i + 1);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // TODO: Tune!
#define Z_GIVE_UP_CATCHING 17.5  // TODO: From config file?
#define Z_RETRY_CATCH 1.0
#define CANDIDATE_TIMEOUT 30   // [s]
#define CATCHING_LOOP_RATE 50  // [Hz]
#define AVG_XY_ERROR_WINDOW_SIZE 13
#define MAX_DELTA_Z 0.5       // [m] --> [m/s]
#define MAX_AVG_XY_ERROR 0.3  // [m]

      grvc::utils::PidController x_pid("x", 0.4, 0.02, 0);  // TODO: class members?
      grvc::utils::PidController y_pid("y", 0.4, 0.02, 0);
      grvc::utils::PidController z_pid("z", 0.4, 0.02, 0);

      grvc::utils::CircularBuffer history_xy_errors;
      history_xy_errors.set_size(AVG_XY_ERROR_WINDOW_SIZE);
      history_xy_errors.fill_with(MAX_AVG_XY_ERROR);
      // unsigned tries_counter = 0;  // TODO: as feedback?
      ros::Duration timeout(CANDIDATE_TIMEOUT);
      ros::Rate loop_rate(CATCHING_LOOP_RATE);
      ros::Time start_time = ros::Time::now();

      while (true)
      {
         geometry_msgs::Point target_position;
         ros::Duration since_last_point, since_last_candidate;
         if (!object_sensed_)
         {
            since_last_point = ros::Time::now() - start_time;

            target_position.x = _goal->lower_bound.x + 5;
            target_position.y = _goal->upper_bound.y - i * offset;
            target_position.z = _goal->altitude.data;
         }
         else
         {
            if (matched_candidate_.pose.pose.position.x > _goal->lower_bound.x &&
                matched_candidate_.pose.pose.position.x < _goal->upper_bound.x &&
                matched_candidate_.pose.pose.position.y > _goal->lower_bound.y &&
                matched_candidate_.pose.pose.position.y < _goal->upper_bound.y &&
                matched_candidate_.pose.pose.position.z > _goal->lower_bound.z &&
                matched_candidate_.pose.pose.position.z < _goal->upper_bound.z)
            {
               since_last_candidate = ros::Time::now() - matched_candidate_.header.stamp;
               
               // ROS_INFO("since_last_candidate = %lf, timeout = %lf", since_last_candidate.toSec(), timeout.toSec());
               target_position.x = matched_candidate_.pose.pose.position.x - 3;
               target_position.y = matched_candidate_.pose.pose.position.y;
               target_position.z = matched_candidate_.pose.pose.position.z;
            }
            else
            {
               object_sensed_ = false;
               start_time     = ros::Time::now();
            }
         }
         if(since_last_candidate > timeout) {object_sensed_ = false;}
         if (since_last_point > timeout)
         {
            object_sensed_ = false;
            start_time     = ros::Time::now();
            i--;
            if (i == 0)
            {
               ROS_WARN("Target not found, search target action failed!");
               result.message = "Search timed out";
               search_target_server_.setAborted(result);
               return;
            }
         }

         tf::Quaternion quat;
         tf::quaternionMsgToTF(ual_->pose().pose.orientation, quat);
         double roll, pitch, yaw;
         tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
         double yaw_rate                = -yaw;
         geometry_msgs::Pose drone_pose = ual_->pose().pose;
         // Send target_position  // TODO: find equivalent!
         geometry_msgs::TwistStamped velocity;
         velocity.header.stamp    = ros::Time::now();
         velocity.header.frame_id = "map";
         velocity.twist.linear.x =
             x_pid.control_signal(target_position.x - drone_pose.position.x, 1.0 / CATCHING_LOOP_RATE);
         velocity.twist.linear.y =
             y_pid.control_signal(target_position.y - drone_pose.position.y, 1.0 / CATCHING_LOOP_RATE);
         velocity.twist.linear.z =
             z_pid.control_signal(target_position.z - drone_pose.position.z, 1.0 / CATCHING_LOOP_RATE);
         velocity.twist.angular.z = yaw_rate;

         ual_->setVelocity(velocity);
         
         // ROS_INFO_STREAM("Candidate position = ["+std::to_string(matched_candidate_.pose.pose.position.x)+  ", "+std::to_string(matched_candidate_.pose.pose.position.y)+  ", "+std::to_string(matched_candidate_.pose.pose.position.z)+  "]");
         // ROS_INFO_STREAM("target_position = ["+std::to_string(target_position.x)+", "+std::to_string(target_position.y)+", "+std::to_string(target_position.z)+"] target angle = x");
         // ROS_INFO_STREAM("velocity = ["+std::to_string(velocity.twist.linear.x)+", "+std::to_string(velocity.twist.linear.y)+", "+std::to_string(velocity.twist.linear.z)+"]");
         // If we're too high, give up TODO: use _goal->z?
         if (object_sensed_depth_)
         {
            if (depth_detection_.point.x - matched_candidate_.pose.pose.position.x < 1.5 &&
                depth_detection_.point.y - matched_candidate_.pose.pose.position.y < 1.5 &&
                depth_detection_.point.z - matched_candidate_.pose.pose.position.z < 1.5)
            {
               result.message = "Drone found in depth image";
               search_target_server_.setSucceeded(result);
               break;
            }
            else
            {
               object_sensed_depth_ = false;
            }
            // TODO: Find equivalent
            // mbzirc_scheduler::SetTargetStatus target_status_call;
            // target_status_call.request.target_id = target_.target_id;
            // target_status_call.request.target_status = mbzirc_scheduler::SetTargetStatus::Request::LOST;
            // if (!target_status_client_.call(target_status_call)) {
            //   ROS_ERROR("Error setting target status to LOST in UAV_%d", uav_id_);
            // }
            // hover_position_waypoint_ = ual_.pose();
         }

         // TODO: Review this frequency!
         loop_rate.sleep();
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // ual_->takeOff(_goal->waypoint.pose.position.z - ual_->pose().pose.position.z, true);  // TODO: timeout?
      // preempt? ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt? ual_->goToWaypoint(_goal->waypoint);
      // // TODO: timeout? preempt?
      sensed_sub_.shutdown();
      depth_sensed_sub_.shutdown();
      object_sensed_depth_ = false;
   }

   void trackCallback(const mbzirc_comm_objs::TrackGoalConstPtr &_goal)
   {
      // ROS_INFO("Track!");
      // mbzirc_comm_objs::PickFeedback feedback;
      mbzirc_comm_objs::TrackResult result;

      if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO)
      {
         ROS_WARN("UAL is not flying auto!");
         track_server_.setAborted(result);
         return;
      }

      ros::NodeHandle nh;
      ros::Subscriber depth_sensed_sub_ =
          nh.subscribe("drone_detector/detection_relative_points", 1, &UalActionServer::depthDetectionCallback, this);
      ros::Subscriber sensed_sub_ =
          nh.subscribe("color_detector/detection_relative_points", 1, &UalActionServer::sensedObjectsCallback, this);

      ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // TODO: Tune!
#define DETECTION_TIMEOUT 5    // [s]
#define TRACKING_LOOP_RATE 10  // [Hz]

      grvc::utils::PidController x_pid("x", 0.4, 0.02, 0);  // TODO: class members?
      grvc::utils::PidController y_pid("y", 0.4, 0.02, 0);
      grvc::utils::PidController z_pid("z", 0.4, 0.02, 0);

      ros::Duration timeout(DETECTION_TIMEOUT);
      ros::Rate loop_rate(TRACKING_LOOP_RATE);
      ros::Time last_time = ros::Time::now();
      geometry_msgs::TwistStamped last_velocity;
      geometry_msgs::Point target_position;
      while (true)
      {
         geometry_msgs::TwistStamped velocity;
         ros::Duration since_last_candidate;
         if (!object_sensed_depth_)
         {
            since_last_candidate = ros::Time::now() - last_time;

            velocity = last_velocity;
         }
         else
         {
            target_position = depth_detection_.point;
            last_time       = ros::Time::now();

            double yaw_setpoint = atan2(target_position.y, target_position.x);

            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(ual_->pose().pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            double yaw_rate = (-90. + yaw_setpoint - yaw) / 90.;

            // Send target_position  // TODO: find equivalent!
            velocity.header.stamp    = ros::Time::now();
            velocity.header.frame_id = "map";
            velocity.twist.linear.x  = x_pid.control_signal(target_position.x, 1.0 / TRACKING_LOOP_RATE);
            velocity.twist.linear.y  = y_pid.control_signal(target_position.y, 1.0 / TRACKING_LOOP_RATE);
            velocity.twist.linear.z  = z_pid.control_signal(target_position.z, 1.0 / TRACKING_LOOP_RATE);
            velocity.twist.angular.z = yaw_rate;
         }

         target_position = matched_candidate_.relative_position;
         if (since_last_candidate > timeout)
         {
            object_sensed_ = false;
            ROS_WARN("Target lost, tracking failed!");
            result.message = "Tracking lost";
            track_server_.setAborted(result);
            return;
         }

         ual_->setVelocity(velocity);
         last_velocity = velocity;

         // TODO: termination condition of eight estimated
         bool eight_detected = false;
         if (eight_detected)
         {
            result.message = "Eight detected";
            track_server_.setSucceeded(result);
            break;
         }

         // TODO: Review this frequency!
         loop_rate.sleep();
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // ual_->takeOff(_goal->waypoint.pose.position.z - ual_->pose().pose.position.z, true);  // TODO: timeout?
      // preempt? ual_->goToWaypoint(_goal->waypoint);  // TODO: timeout? preempt? ual_->goToWaypoint(_goal->waypoint);
      // // TODO: timeout? preempt?
      sensed_sub_.shutdown();
   }

   void landCallback(const mbzirc_comm_objs::LandGoalConstPtr &_goal)
   {
      // ROS_INFO("Land!");
      // mbzirc_comm_objs::LandFeedback feedback;
      mbzirc_comm_objs::LandResult result;

      if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO)
      {
         ROS_WARN("UAL is not flying auto!");
         land_server_.setAborted(result);
         return;
      }

      ual_->land(true);

      land_server_.setSucceeded(result);
   }

   void moveInCirclesCallback(const mbzirc_comm_objs::MoveInCirclesGoalConstPtr &_goal)
   {
      mbzirc_comm_objs::MoveInCirclesFeedback feedback;
      mbzirc_comm_objs::MoveInCirclesResult result;

      if (ual_->state().state != uav_abstraction_layer::State::FLYING_AUTO)
      {
         result.message = "UAL is not flying auto!";
         move_in_circles_server_.setAborted(result);
         return;
      }

      if (_goal->horizontal_distance < 1e-2)
      {
         result.message = "horizontal_distance must be greater than 1cm!";
         move_in_circles_server_.setAborted(result);
         return;
      }

      if (fabs(_goal->horizontal_velocity) < 1e-2)
      {
         result.message = "horizontal_velocity absolute value must be grater than 1cm/s!";
         move_in_circles_server_.setAborted(result);
         return;
      }

      geometry_msgs::TransformStamped robot_transform;
      std::string robot_frame_id = ual_->transform().child_frame_id;
      try
      {
         robot_transform = tf_buffer_.lookupTransform(_goal->origin.header.frame_id, robot_frame_id, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
         result.message = ex.what();
         move_in_circles_server_.setAborted(result);
         return;
      }
      float delta_x = robot_transform.transform.translation.x - _goal->origin.pose.position.x;
      float delta_y = robot_transform.transform.translation.y - _goal->origin.pose.position.y;
      float theta   = atan2(delta_y, delta_x);

      float frequency = 10;  // [Hz]
      ros::Rate loop_rate(frequency);
      float delta_theta       = _goal->horizontal_velocity / (_goal->horizontal_distance * frequency);
      int max_iteration_count = _goal->revolutions_count * ceil(2 * M_PI / fabs(delta_theta));
      int iteration_count     = 0;

      while (ros::ok())
      {
         if (move_in_circles_server_.isPreemptRequested() || !ros::ok())
         {
            move_in_circles_server_.setPreempted();
            return;
         }
         geometry_msgs::PoseStamped circle_reference = _goal->origin;
         circle_reference.pose.position.x += _goal->horizontal_distance * cos(theta);
         circle_reference.pose.position.y += _goal->horizontal_distance * sin(theta);
         circle_reference.pose.position.z += _goal->vertical_distance;
         float half_yaw                      = 0.5 * (theta + M_PI);
         circle_reference.pose.orientation.x = 0;
         circle_reference.pose.orientation.y = 0;
         circle_reference.pose.orientation.z = sin(half_yaw);
         circle_reference.pose.orientation.w = cos(half_yaw);
         if (iteration_count == 0)
         {
            ual_->goToWaypoint(circle_reference, true);
         }
         else
         {
            ual_->setPose(circle_reference);
         }
         feedback.current_target = circle_reference;
         move_in_circles_server_.publishFeedback(feedback);
         if (iteration_count++ >= max_iteration_count)
         {
            break;
         }
         theta += delta_theta;
         loop_rate.sleep();
      }
      move_in_circles_server_.setSucceeded(result);
   }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_action_server");

   UalActionServer uav_action_server;
   ros::MultiThreadedSpinner spinner(2);
   spinner.spin();

   return 0;
}
