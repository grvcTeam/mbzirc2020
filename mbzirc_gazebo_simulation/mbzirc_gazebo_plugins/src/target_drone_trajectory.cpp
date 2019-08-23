#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <ros/console.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Time begin;
ros::Timer timer;
ros::Publisher pub;
ros::ServiceClient arm_service;
ros::ServiceClient mode_service;
ros::ServiceClient model_state_service;

gazebo_msgs::GetModelState::Request getmodelstate_req;
gazebo_msgs::GetModelState::Response getmodelstate_resp;
mavros_msgs::CommandBool::Request arm_service_req;
mavros_msgs::CommandBool::Response arm_service_resp;
mavros_msgs::SetMode::Request mode_service_req;
mavros_msgs::SetMode::Response mode_service_resp;

geometry_msgs::Pose Init_Drone_Pose;

float a, b, speed, scale_factor;
float offset_x, offset_y, offset_z, max_balloons_height;
std::string model_name;

void getInputParams(ros::NodeHandle& nh) {
   /*      Trajectory params definition
           ----------------------------
     x = offset_x + a * sin(t/scale_factor)
     y = offset_y + a * sin(t/scale_factor) * cos(t/scale_factor)
     z = offset_z + b * sin(t/scale_factor)

     To adjust desire speed: scale_factor
     scale_factor = a / speed

     To keep the trajectory centered on (50,30)
     offset_x = 50 - Init_Drone_Pose(x)    --> read from Gazebo. See Main
     offset_y = 30 - Init_Drone_Pose(y)    --> read from Gazebo. See Main
     The offset of z axis will be calculated depending on balloons position
     offset_z = (20 - max_balloons_height)/2    --> got from challenge1.launch

     where: offset,a,b [m]  |  speed [m/s]
 */

   if (nh.getParam("speed", speed) && nh.getParam("a", a) && nh.getParam("b", b) &&
       nh.getParam("model_name", model_name)) {
      ROS_INFO("Parameters received --> a:[%f] b:[%f] speed:[%f]m/s model_name:[%s]", a, b, speed, model_name.c_str());
      if (speed > 10) {
         ROS_WARN("Maximum speed exceeded. Cropped to 10m/s ");
         speed = 10;
      }
      scale_factor = a / speed;
      offset_x     = 50 - Init_Drone_Pose.position.x;
      offset_y     = 30 - Init_Drone_Pose.position.y;
      offset_z     = (20 - max_balloons_height) / 2;
   } else {
      ROS_ERROR("Parameters not received!");
   }
}

void poseCB(const ros::TimerEvent& event) {
   geometry_msgs::PoseStamped pose_local;
   ros::Time current = ros::Time::now();
   ros::Duration d;
   double t;

   // get instant time to generate drone trajectory
   d = current - begin;
   t = d.toSec();

   // set drone position
   pose_local.header.seq        = 0;
   pose_local.header.stamp.sec  = 0;
   pose_local.header.stamp.nsec = 0;
   pose_local.header.frame_id   = "";

   pose_local.pose.orientation.x = 0;
   pose_local.pose.orientation.y = 0;
   pose_local.pose.orientation.z = 0;
   pose_local.pose.orientation.w = 1;

   pose_local.pose.position.x = offset_x + a * sin(t / scale_factor);
   pose_local.pose.position.y = offset_y + a * sin(t / scale_factor) * cos(t / scale_factor);
   pose_local.pose.position.z = offset_z + b * sin(t / scale_factor);

   // publish drone position
   pub.publish(pose_local);
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "drone_pose");
   ros::NodeHandle nh("~");

   begin = ros::Time::now();

   timer               = nh.createTimer(ros::Duration(0.001), poseCB);
   arm_service         = nh.serviceClient<mavros_msgs::CommandBool>("/target_drone/mavros/cmd/arming");
   mode_service        = nh.serviceClient<mavros_msgs::SetMode>("/target_drone/mavros/set_mode");
   model_state_service = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
   pub                 = nh.advertise<geometry_msgs::PoseStamped>("/target_drone/mavros/setpoint_position/local", 1000);

   // Call get input params
   getInputParams(nh);

   // get drone initial pose
   getmodelstate_req.model_name = model_name;

   ROS_INFO("Calling /gazebo/get_model_state for: %s", model_name.c_str());
   if (model_state_service.call(getmodelstate_req, getmodelstate_resp)) {
      if (getmodelstate_resp.success) {
         Init_Drone_Pose.position = getmodelstate_resp.pose.position;
         ROS_INFO("Got init drone pose. x:%f y:%f", Init_Drone_Pose.position.x, Init_Drone_Pose.position.y);
      } else {
         ROS_WARN("Unable to get init drone pose");
      }
   } else {
      ROS_INFO("Call service failed");
      ROS_ERROR("Fail to connect with gazebo server");
      return 0;
   }

   ros::Duration(2).sleep();

   // arm Drone
   arm_service_req.value = true;

   if (arm_service.call(arm_service_req, arm_service_resp)) {
      if (arm_service_resp.success) {
         ROS_INFO("Drone arming succeed");
      } else {
         ROS_WARN("Drone arming failed");
      }
   } else {
      ROS_INFO("Call service failed");
      ROS_ERROR("Fail to connect with gazebo server");
      return 0;
   }

   // set offboard mode
   mode_service_req.custom_mode = "OFFBOARD";
   mode_service_req.base_mode   = 0;

   if (mode_service.call(mode_service_req, mode_service_resp)) {
      if (mode_service_resp.mode_sent) {
         ROS_INFO("Set mode offboard");
      } else {
         ROS_WARN("Unable to set offboard mode");
      }
   } else {
      ROS_INFO("Call service failed");
      ROS_ERROR("Fail to connect with gazebo server");
      return 0;
   }

   ros::spin();

   return 0;
}
