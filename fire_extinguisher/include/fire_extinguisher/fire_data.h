#ifndef FIRE_DATA_H
#define FIRE_DATA_H

#include <yaml-cpp/yaml.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbzirc_comm_objs/WallList.h>

struct FireData {
    std::string id;
    geometry_msgs::PoseStamped ual_pose;
    sensor_msgs::Range sf11_range;
    mbzirc_comm_objs::WallList wall_list;
};

void operator>>(const YAML::Node& in, FireData& fire_data) {
    // TODO: Check that expected fields do exist
    fire_data.id = in["id"].as<std::string>();
    fire_data.ual_pose.header.frame_id = in["pose_frame"].as<std::string>();
    fire_data.ual_pose.pose.position.x = in["pose"][0].as<double>();
    fire_data.ual_pose.pose.position.y = in["pose"][1].as<double>();
    fire_data.ual_pose.pose.position.z = in["pose"][2].as<double>();
    fire_data.ual_pose.pose.orientation.x = in["pose"][3].as<double>();
    fire_data.ual_pose.pose.orientation.y = in["pose"][4].as<double>();
    fire_data.ual_pose.pose.orientation.z = in["pose"][5].as<double>();
    fire_data.ual_pose.pose.orientation.w = in["pose"][6].as<double>();
    fire_data.sf11_range.range = in["sf11_range"].as<float>();
    fire_data.wall_list.header.frame_id = in["walls_frame"].as<std::string>();
    for (int i = 0; i < in["walls"].size(); i++) {
        mbzirc_comm_objs::Wall wall;
        wall.start[0] = in["walls"][i][0].as<float>();
        wall.start[1] = in["walls"][i][1].as<float>();
        wall.end[0]   = in["walls"][i][2].as<float>();
        wall.end[1]   = in["walls"][i][3].as<float>();
        fire_data.wall_list.walls.push_back(wall);
    }
}

#endif  // FIRE_DATA_H
