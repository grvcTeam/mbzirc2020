#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
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

class FireExtinguisher {
public:

    FireExtinguisher() {
        std::string fires_folder = ros::package::getPath("fire_extinguisher") + "/fires/";
        std::string fires_filename = fires_folder + "fire_default.yaml";  // TODO: from parameter?

        YAML::Node yaml_fires = YAML::LoadFile(fires_filename);
        for (std::size_t i = 0; i < yaml_fires["fires"].size(); i++) {
            FireData fire_data;
            yaml_fires["fires"][i] >> fire_data;

            std::cout << "\n index: " << i<< '\n';
            std::cout << fire_data.id << '\n';
            std::cout << fire_data.ual_pose << '\n';
            std::cout << fire_data.sf11_range << '\n';
            std::cout << fire_data.wall_list << '\n';
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fire_extinguisher_node");

    FireExtinguisher fire_extinguisher;
    ros::spin();

    return 0;
}
