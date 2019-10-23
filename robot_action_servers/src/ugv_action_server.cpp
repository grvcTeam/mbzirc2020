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
#include <mbzirc_comm_objs/GoToAction.h>
#include <mbzirc_comm_objs/PickAction.h>
#include <mbzirc_comm_objs/PlaceAction.h>
#include <mbzirc_comm_objs/RobotDataFeed.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

#include <mbzirc_comm_objs/GripperAttached.h>
#include <mbzirc_comm_objs/Magnetize.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>




// #include <handy_tools/pid_controller.h>
// #include <handy_tools/circular_buffer.h>

class UgvActionServer {
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs
  actionlib::SimpleActionServer<mbzirc_comm_objs::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::PickAction> pick_server_;
  actionlib::SimpleActionServer<mbzirc_comm_objs::PlaceAction> place_server_;


  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener *tf_listener_;

  ros::Timer data_feed_timer_;
  ros::Publisher data_feed_pub_;
  nav_msgs::Odometry robot_pose_;

  bool gripper_attached_ = false;

public:

  UgvActionServer():
    go_to_server_(nh_, "go_to_action", boost::bind(&UgvActionServer::goToCallback, this, _1), false),
    pick_server_(nh_, "pick_action", boost::bind(&UgvActionServer::pickCallback, this, _1), false),
    place_server_(nh_, "place_action", boost::bind(&UgvActionServer::placeCallback, this, _1), false)
    {

    // tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    data_feed_timer_ = nh_.createTimer(ros::Duration(0.1), &UgvActionServer::publishDataFeed, this);  // TODO: frequency?
    data_feed_pub_ = nh_.advertise<mbzirc_comm_objs::RobotDataFeed>("data_feed", 1);  // TODO: namespacing?
    // TODO: start servers only when needed?
    // TODO: make servers separated classes?
    
    go_to_server_.start();
    pick_server_.start();
    //place_server_.start();

  }


  void publishDataFeed(const ros::TimerEvent&) {
    ros::NodeHandle nh;
    ros::Subscriber odom_pose_ = nh.subscribe("odometry/filtered", 1, &UgvActionServer::robotPoseCallback, this);
    mbzirc_comm_objs::RobotDataFeed data_feed;

    data_feed.pose.header = robot_pose_.header;
    data_feed.pose.pose = robot_pose_.pose.pose;
    data_feed_pub_.publish(data_feed);
  }

    void robotPoseCallback(const nav_msgs::Odometry& msg) {
      robot_pose_ = msg;
    }


  void goToCallback(const mbzirc_comm_objs::GoToGoalConstPtr &_goal) {


    //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_ac("move_base", true);
    mbzirc_comm_objs::GoToResult result;

    while(!movebase_ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");

    }
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose=_goal->waypoint;

    movebase_ac.sendGoal(goal);

    movebase_ac.waitForResult();

    if(movebase_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Move base SUCCEEDED");
      go_to_server_.setSucceeded(result); 
    }else{
      ROS_INFO("Move base FAILED");
      go_to_server_.setAborted(result);
    }
    // ROS_INFO("Go to!");
    // mbzirc_comm_objs::GoToFeedback feedback;
  }



  void attachedCallback(const mbzirc_comm_objs::GripperAttachedConstPtr& msg) {
    gripper_attached_ = msg->attached;
  }

  void pickCallback(const mbzirc_comm_objs::PickGoalConstPtr &_goal) {
    // ROS_INFO("Pick!");
    // mbzirc_comm_objs::PickFeedback feedback;
    mbzirc_comm_objs::PickResult result;

    ros::NodeHandle nh;
    ros::Publisher  color_detect = nh_.advertise<std_msgs::String>("/color2detect", 1); 
    ros::Publisher  pregrasp = nh_.advertise<std_msgs::String>("/mir_manipulation/pregrasp_planner_pipeline/event_in", 1); 
    ros::Publisher  pregrasp_2 = nh_.advertise<std_msgs::String>("/move_arm_planned_motion/event_in", 1); 
    ros::Subscriber attached_sub_ = nh.subscribe("attached", 1, &UgvActionServer::attachedCallback, this);
    ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    ros::Duration(1.0).sleep();  // TODO: tune! needed for sensed_sub?

    // std::vector<double> joint_values = {-1.5226200262652796, -1.8621331653990687, 0.6166723410235804, -0.6139758390239258, -1.5802353064166468, -2.371239248906271};
    // group.setJointValueTarget(joint_values);
    // group.move();

    // Just for safety measurements
    mbzirc_comm_objs::Magnetize magnetize_srv;
    magnetize_srv.request.magnetize = true;
    if (!magnetize_client.call(magnetize_srv)) {
      ROS_ERROR("Failed to call [magnetize] service");
    }

    color_detect.publish(_goal->color);
    std_msgs::String message;
    message.data = "e_start";
    pregrasp.publish(message);
    pregrasp_2.publish(message);
    ros::Duration(5.0).sleep();

    geometry_msgs::PoseStamped next_pose;
    while (gripper_attached_ != true)
    {
      next_pose = group.getCurrentPose();
      next_pose.pose.position.z -= 0.01;
      group.setPoseTarget(next_pose);
      group.move();
    }
    
    next_pose = group.getCurrentPose();
    next_pose.pose.position.z += 0.05;
    group.setPoseTarget(next_pose);
    group.move();
//check if pick was succesful and setAborted accordingly
    pick_server_.setSucceeded(result); 
    }


  void placeCallback(const mbzirc_comm_objs::PlaceGoalConstPtr &_goal) {
    // ROS_INFO("Place!");
    // mbzirc_comm_objs::PlaceFeedback feedback;
    mbzirc_comm_objs::PlaceResult result;
    ros::NodeHandle nh;
    ros::ServiceClient magnetize_client = nh.serviceClient<mbzirc_comm_objs::Magnetize>("magnetize");
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    mbzirc_comm_objs::Magnetize magnetize_srv;

    geometry_msgs::PoseStamped drop_pose;
    drop_pose = group.getCurrentPose();
    drop_pose.pose.position.z -= 0.25;
    group.setPoseTarget(drop_pose);
    group.move();

    magnetize_srv.request.magnetize = false;
    if (!magnetize_client.call(magnetize_srv)) {
      ROS_ERROR("Failed to call [magnetize] service");  // TODO: retry?
    }

    place_server_.setSucceeded(result);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_action_server");

  UgvActionServer uav_action_server;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}