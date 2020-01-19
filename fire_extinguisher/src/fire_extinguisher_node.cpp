#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_comm_objs/WallList.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <handy_tools/pid_controller.h>
#include <fire_extinguisher/fire_data.h>
#include <scan_passage_detection/wall_utils.h>

#define CONTROL_PERIOD 0.05

class FireExtinguisher {
public:

    FireExtinguisher() {
        std::string fires_folder = ros::package::getPath("fire_extinguisher") + "/fires/";
        std::string fires_filename = fires_folder + "fire_default.yaml";  // TODO: from parameter?

        x_pid_ = new grvc::utils::PidController("x", 0.4, 0.02, 0);
        y_pid_ = new grvc::utils::PidController("y", 0.4, 0.02, 0);
        z_pid_ = new grvc::utils::PidController("z", 0.4, 0.02, 0);
        yaw_pid_ = new grvc::utils::PidController("yaw", 0.4, 0.02, 0);

        YAML::Node yaml_fires = YAML::LoadFile(fires_filename);
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

        pose_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, &FireExtinguisher::poseCallback, this);
        range_sub_ = n_.subscribe<sensor_msgs::Range>("sf11", 1, &FireExtinguisher::rangeCallback, this);
        walls_sub_ = n_.subscribe<mbzirc_comm_objs::WallList>("walls", 1, &FireExtinguisher::wallsCallback, this);
        control_timer_ = n_.createTimer(ros::Duration(CONTROL_PERIOD), &FireExtinguisher::controlCallback, this);  // TODO: frequency from param!
        velocity_pub_ = n_.advertise<geometry_msgs::TwistStamped>("ual/set_velocity", 1);
        marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
        goto_client_ = n_.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    }

    void track() {
        track_timer_ = n_.createTimer(ros::Duration(0.1), &FireExtinguisher::trackCallback, this);  // TODO: frequency from param!
    }

    void trackCallback(const ros::TimerEvent& event) {
        if (!has_ual_pose_)   { return; }
        if (!has_sf11_range_) { return; }
        if (!has_wall_list_)  { return; }

        ROS_ERROR("sf11_range_.range: %lf", sf11_range_.range);

        // TODO: target criteria
        mbzirc_comm_objs::Wall target_wall = closestWall(wall_list_);
        // mbzirc_comm_objs::Wall target_wall = largestWall(wall_list_);

        double x_s = target_wall.start[0];
        double y_s = target_wall.start[1];
        double x_e = target_wall.end[0];
        double y_e = target_wall.end[1];
        double length = sqrt(squaredLength(target_wall));
        double distance = sqrt(squaredDistanceToSegment(0, 0, x_s, y_s, x_e, y_e));
        double dx = x_e - x_s;
        double dy = y_e - y_s;
        double yaw = atan2(dy, dx);

        ROS_ERROR("wall: [%lf, %lf] [%lf, %lf]", x_s, y_s, x_e, y_e);
        ROS_ERROR("length: %lf", length);
        ROS_ERROR("distance: %lf", distance);
        ROS_ERROR("yaw - pi/2: %lf", yaw - 0.5*M_PI);
        // std::cout << current_wall;

        double current_yaw = 2 * atan2(ual_pose_.pose.orientation.z, ual_pose_.pose.orientation.w);
        double yaw_error = target_yaw_ - current_yaw;

    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg) {
        ual_pose_ = *_msg;
        has_ual_pose_ = true;
    }

    void rangeCallback(const sensor_msgs::Range::ConstPtr& _msg) {
        sf11_range_ = *_msg;
        has_sf11_range_ = true;
    }

    void wallsCallback(const mbzirc_comm_objs::WallList::ConstPtr& _msg) {
        wall_list_ = *_msg;
        has_wall_list_ = true;
    }

    void extinguish(const std::string& fire_id) {
        target_is_set_ = false;
        target_fire_ = fire_map[fire_id];  // TODO: check existence!

        largest_wall_ = largestWall(target_fire_.wall_list);
        closest_wall_ = closestWall(target_fire_.wall_list);
        target_yaw_ = 2 * atan2(target_fire_.ual_pose.pose.orientation.z, target_fire_.ual_pose.pose.orientation.w);

        uav_abstraction_layer::GoToWaypoint goto_srv;
        goto_srv.request.waypoint = target_fire_.ual_pose;
        goto_srv.request.blocking = true;
        if (goto_client_.call(goto_srv)) {
            target_is_set_ = true;
        }
    }

    void controlCallback(const ros::TimerEvent& event) {
        if (!target_is_set_)  { return; }
        if (!has_ual_pose_)   { return; }
        if (!has_sf11_range_) { return; }
        if (!has_wall_list_)  { return; }

        double z_error = target_fire_.sf11_range.range - sf11_range_.range;
        // ROS_ERROR("z: %lf", z_error);

        // TODO: target criteria
        mbzirc_comm_objs::Wall target_wall = closest_wall_;
        mbzirc_comm_objs::Wall current_wall = closestWall(wall_list_);
        // mbzirc_comm_objs::Wall target_wall = largest_wall_;
        // mbzirc_comm_objs::Wall current_wall = largestWall(wall_list_);

        visualization_msgs::MarkerArray marker_array;
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        marker_array.markers.push_back(getLineMarker(target_wall, wall_list_.header.frame_id, color, 0, "wall_control"));

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
        marker_array.markers.push_back(getLineMarker(current_wall, wall_list_.header.frame_id, color, 1, "wall_control"));
        marker_pub_.publish(marker_array);

        // ROS_ERROR("start: %lf, %lf", x_start_error, y_start_error);
        // ROS_ERROR("end: %lf, %lf", x_end_error, y_end_error);
        // ROS_ERROR("xy: %lf, %lf", x_error, y_error);
        // std::cout << current_wall;

        double current_yaw = 2 * atan2(ual_pose_.pose.orientation.z, ual_pose_.pose.orientation.w);
        double yaw_error = target_yaw_ - current_yaw;

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = ros::Time::now();
        velocity.header.frame_id = wall_list_.header.frame_id;
        velocity.twist.linear.x = x_pid_->control_signal(-x_error, CONTROL_PERIOD);
        velocity.twist.linear.y = y_pid_->control_signal(-y_error, CONTROL_PERIOD);
        velocity.twist.linear.z = z_pid_->control_signal(z_error, CONTROL_PERIOD);
        velocity.twist.angular.z = yaw_pid_->control_signal(yaw_error, CONTROL_PERIOD);

        // std::cout << velocity << '\n';
        velocity_pub_.publish(velocity);
    }

protected:
    std::map<std::string, FireData> fire_map;
    ros::NodeHandle n_;
    ros::Subscriber pose_sub_;
    ros::Subscriber range_sub_;
    ros::Subscriber walls_sub_;
    ros::Timer control_timer_;
    ros::Timer track_timer_;
    ros::Publisher velocity_pub_;
    ros::Publisher marker_pub_;
    ros::ServiceClient goto_client_;

    grvc::utils::PidController *x_pid_;
    grvc::utils::PidController *y_pid_;
    grvc::utils::PidController *z_pid_;
    grvc::utils::PidController *yaw_pid_;

    geometry_msgs::PoseStamped ual_pose_;
    sensor_msgs::Range sf11_range_;
    mbzirc_comm_objs::WallList wall_list_;
    bool has_ual_pose_ = false;
    bool has_sf11_range_ = false;
    bool has_wall_list_ = false;
    FireData target_fire_;
    bool target_is_set_ = false;
    mbzirc_comm_objs::Wall closest_wall_;
    mbzirc_comm_objs::Wall largest_wall_;
    double target_yaw_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fire_extinguisher_node");

    FireExtinguisher fire_extinguisher;
    fire_extinguisher.track();
    // fire_extinguisher.extinguish("default_0");
    ros::spin();

    return 0;
}