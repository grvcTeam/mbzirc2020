#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_comm_objs/WallList.h>
#include <scan_passage_detection/wall_utils.h>

class WallTracker {
public:

    WallTracker() {
        pose_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, &WallTracker::poseCallback, this);
        range_sub_ = n_.subscribe<sensor_msgs::Range>("sf11", 1, &WallTracker::rangeCallback, this);
        walls_sub_ = n_.subscribe<mbzirc_comm_objs::WallList>("walls", 1, &WallTracker::wallsCallback, this);
        marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
        track_timer_ = n_.createTimer(ros::Duration(0.1), &WallTracker::trackCallback, this);  // TODO: frequency from param!
    }

    void trackCallback(const ros::TimerEvent& event) {
        if (!has_ual_pose_)   { return; }
        if (!has_sf11_range_) { return; }
        if (!has_wall_list_)  { return; }

        // ROS_ERROR("sf11_range_.range: %lf", sf11_range_.range);

        // TODO: target criteria
        mbzirc_comm_objs::Wall target_wall = closestWall(wall_list_);
        // mbzirc_comm_objs::Wall target_wall = largestWall(wall_list_);
        double desired_distance = 2.0;  // desired distance (to rplidar)

        double x_s = target_wall.start[0];
        double y_s = target_wall.start[1];
        double x_e = target_wall.end[0];
        double y_e = target_wall.end[1];
        double length = sqrt(squaredLength(target_wall));
        double distance = sqrt(squaredDistanceToSegment(0, 0, x_s, y_s, x_e, y_e));
        double dx = x_e - x_s;
        double dy = y_e - y_s;
        double dyaw = atan2(dy, dx) - 0.5*M_PI;  // if 0, uav is perpendicular to wall
        double yaw_error = fabs(dyaw);
        double distance_error = fabs(desired_distance - distance);

        // ROS_ERROR("wall: [%lf, %lf] [%lf, %lf]", x_s, y_s, x_e, y_e);
        // ROS_ERROR("length: %lf", length);
        // ROS_ERROR("distance: %lf", distance);
        // ROS_ERROR("dyaw: %lf", dyaw);
        // std::cout << current_wall;

        double reference_length = length + 2.0;
        mbzirc_comm_objs::Wall reference_wall;
        reference_wall.start[0] =  desired_distance;
        reference_wall.start[1] = -0.5 * reference_length;
        reference_wall.end[0]   =  desired_distance;
        reference_wall.end[1]   =  0.5 * reference_length;
        double x_med = 0.5 * (x_s + x_e);
        double y_med = 0.5 * (y_s + y_e);

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker text_marker;
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;

        int marker_id = 0;
        marker_array.markers.push_back(getLineMarker(reference_wall, wall_list_.header.frame_id, color, marker_id++, "tracked_wall"));

        double error = distance_error + yaw_error;
        double threshold = 1.0;
        double rel_error = error / threshold;
        color.r = (rel_error > 1.0)? 1.0: rel_error;
        color.g = (rel_error > 1.0)? 0.0: 1 - rel_error;
        color.b = 0.0;
        color.a = 1.0;
        marker_array.markers.push_back(getLineMarker(target_wall, wall_list_.header.frame_id, color, marker_id++, "tracked_wall"));

        text_marker.header.frame_id = wall_list_.header.frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "tracked_wall";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = x_med;
        text_marker.pose.position.y = y_med;
        text_marker.pose.position.z = -0.5;
        text_marker.scale.z = 0.5;
        text_marker.color = color;
        text_marker.lifetime = ros::Duration(0.1);
        text_marker.text = "z = " + std::to_string(sf11_range_.range);
        marker_array.markers.push_back(text_marker);

        text_marker.header.frame_id = wall_list_.header.frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "tracked_wall";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = x_s;
        text_marker.pose.position.y = y_s;
        text_marker.pose.position.z = -0.5;
        text_marker.scale.z = 0.1;
        text_marker.color = color;
        text_marker.lifetime = ros::Duration(0.1);
        text_marker.text = "s(" + std::to_string(x_s) + ", " + std::to_string(y_s) + ")";
        marker_array.markers.push_back(text_marker);

        text_marker.id = marker_id++;
        text_marker.pose.position.x = x_e;
        text_marker.pose.position.y = y_e;
        text_marker.text = "e(" + std::to_string(x_e) + ", " + std::to_string(y_e) + ")";
        marker_array.markers.push_back(text_marker);

        text_marker.id = marker_id++;
        text_marker.pose.position.x = x_med;
        text_marker.pose.position.y = y_med;
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.5;
        text_marker.text = "l = " + std::to_string(length);
        marker_array.markers.push_back(text_marker);

        text_marker.id = marker_id++;
        text_marker.pose.position.x = x_med;
        text_marker.pose.position.y = y_med;
        text_marker.pose.position.z += 1.0;
        text_marker.text = "dyaw = " + std::to_string(dyaw);
        marker_array.markers.push_back(text_marker);

        text_marker.id = marker_id++;
        text_marker.pose.position.x = x_med;
        text_marker.pose.position.y = y_med;
        text_marker.pose.position.z += 1.0;
        text_marker.text = "d = " + std::to_string(distance);
        marker_array.markers.push_back(text_marker);

        marker_pub_.publish(marker_array);
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

protected:
    ros::NodeHandle n_;
    ros::Subscriber pose_sub_;
    ros::Subscriber range_sub_;
    ros::Subscriber walls_sub_;
    ros::Timer track_timer_;
    ros::Publisher marker_pub_;

    geometry_msgs::PoseStamped ual_pose_;
    sensor_msgs::Range sf11_range_;
    mbzirc_comm_objs::WallList wall_list_;
    bool has_ual_pose_ = false;
    bool has_sf11_range_ = false;
    bool has_wall_list_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_tracker_node");

    WallTracker wall_tracker;
    ros::spin();

    return 0;
}
