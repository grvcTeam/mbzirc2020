#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

class Estimator {
public:
    Estimator(double _frequency = 1.0) {
        if (_frequency <= 0) {
            ROS_ERROR("Estimator: Trying to set frequency to invalid value [%lf], using 1Hz instead", _frequency);
            _frequency = 1.0;
        }
        ros::NodeHandle nh;
        estimated_pub_ = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("estimated_objects", 1);
        estimation_timer_ = nh.createTimer(ros::Duration(1.0/_frequency), &Estimator::estimateCallback, this);
    }

protected:
    void estimateCallback(const ros::TimerEvent& event) {
        estimated_pub_.publish(targets_);
    }

    ros::Timer estimation_timer_;
    ros::Publisher estimated_pub_;
    mbzirc_comm_objs::ObjectDetectionList targets_;
};

float squared_distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    return dx*dx + dy*dy + dz*dz;
}

class SimpleClusteringEstimator: public Estimator {
public:

    SimpleClusteringEstimator(double _frequency, const std::string& _uav_ns): Estimator(_frequency) {
        ros::NodeHandle nh;

        int max_uav_count = 3;  // Even if uavs does not exist, subscribing makes no harm
        for (int i = 1; i < max_uav_count + 1; i++) {
            std::string sensed_topic = _uav_ns + "_" + std::to_string(i) + "/sensed_objects";
            sensed_sub_.push_back(nh.subscribe(sensed_topic, 1, &SimpleClusteringEstimator::updateCallback, this));
        }
    }

protected:

    void updateCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) {
        for (auto sensed: msg->objects) {
            bool first_time_sensed = true;
            for (auto &target: targets_.objects) {
                if (squared_distance(sensed.pose.pose.position, target.pose.pose.position) < squared_distance_th_) {
                    // TODO: Supose all object poses are defined in the same frame_id!
                    float sensed_radius = 0.5 * std::max(fabs(sensed.scale.x), fabs(sensed.scale.y));
                    if (sensed_radius > 0.01) {  // threshold: 1 [cm]
                        float target_radius = 0.5 * std::max(fabs(target.scale.x), fabs(target.scale.y));
                        float x_max = std::max(sensed.pose.pose.position.x + sensed_radius, target.pose.pose.position.x + target_radius);
                        float y_max = std::max(sensed.pose.pose.position.y + sensed_radius, target.pose.pose.position.y + target_radius);
                        float x_min = std::min(sensed.pose.pose.position.x - sensed_radius, target.pose.pose.position.x - target_radius);
                        float y_min = std::min(sensed.pose.pose.position.y - sensed_radius, target.pose.pose.position.y - target_radius);

                        float x_scale = x_max - x_min;
                        float y_scale = y_max - y_min;
                        float z_scale = std::max(sensed.scale.z, target.scale.z);  // Supose all objects are at the same height

                        float x_new = x_min + 0.5 * x_scale;
                        float y_new = y_min + 0.5 * y_scale;
                        float z_new = 0.5 * z_scale;

                        // TODO: Something with covariance?
                        target.pose.pose.position.x = x_new;
                        target.pose.pose.position.y = y_new;
                        target.pose.pose.position.z = z_new;
                        target.scale.x = x_scale;
                        target.scale.y = y_scale;
                        target.scale.z = z_scale;
                    }
                    first_time_sensed = false;
                    break;
                }
            }
            if (first_time_sensed) {
                // TODO: id? Use index?
                mbzirc_comm_objs::ObjectDetection new_estimated;
                new_estimated.header = sensed.header;
                new_estimated.type = sensed.type;
                new_estimated.pose.pose.position = sensed.pose.pose.position;
                new_estimated.pose.pose.orientation.w = 1.0;  // Forget orientation...
                new_estimated.pose.covariance = sensed.pose.covariance;
                float xy_scale = std::max(fabs(sensed.scale.x), fabs(sensed.scale.y));
                new_estimated.scale.x = xy_scale;
                new_estimated.scale.y = xy_scale;  // ...and make scale.x = scale.y
                new_estimated.scale.z = sensed.scale.z;
                new_estimated.color = sensed.color;
                targets_.objects.push_back(new_estimated);
            }
        }
    }

    std::vector<ros::Subscriber> sensed_sub_;
    float squared_distance_th_ = 25.0;  // TODO: tune, from params?
};

struct PileData {
    std::string color;
    std::string frame_id;
    float position_x;
    float position_y;
    float position_z;
    float orientation_yaw;
    float scale_x;
    float scale_y;
    float scale_z;

    void print() {
        printf("color: %s, frame_id: %s, position_x: %f, position_y: %f, position_z: %f, orientation_yaw: %f, scale_x: %f, scale_y: %f, scale_z: %f \n", \
                color.c_str(), frame_id.c_str(), position_x, position_y, position_z, orientation_yaw, scale_x, scale_y, scale_z);
    }
};

void operator>>(const YAML::Node& in, PileData& pile_data) {
    // TODO: Check that expected fields do exist
    pile_data.color = in["color"].as<std::string>();
    pile_data.frame_id = in["frame_id"].as<std::string>();
    pile_data.position_x = in["position_x"].as<float>();
    pile_data.position_y = in["position_y"].as<float>();
    pile_data.position_z = in["position_z"].as<float>();
    pile_data.orientation_yaw = in["orientation_yaw"].as<float>();
    pile_data.scale_x = in["scale_x"].as<float>();
    pile_data.scale_y = in["scale_y"].as<float>();
    pile_data.scale_z = in["scale_z"].as<float>();
}

// TODO: Move to some kind of utils lib, as it is repeated
int color_from_string(const std::string& color) {
    int out_color;
    switch(color[0]) {
        case 'R':
        case 'r':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_RED;
            break;
        case 'G':
        case 'g':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_GREEN;
            break;
        case 'B':
        case 'b':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_BLUE;
            break;
        case 'O':
        case 'o':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_ORANGE;
            break;
        default:
        ROS_ERROR("Unknown color %s", color.c_str());
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
    }
    return out_color;
}

class APrioriInfoEstimator: public Estimator {
public:

    APrioriInfoEstimator(double _frequency): Estimator(_frequency) {
        std::string config_folder = ros::package::getPath("target_estimation") + "/config/";
        std::string config_filename = config_folder + "piles.yaml";  // TODO: from parameter?

        YAML::Node yaml_config = YAML::LoadFile(config_filename);
        for (std::size_t i = 0; i < yaml_config["piles"].size(); i++) {
            PileData pile_data;
            yaml_config["piles"][i] >> pile_data;
            // pile_data.print();

            mbzirc_comm_objs::ObjectDetection object;
            object.header.stamp = ros::Time::now();
            object.header.frame_id = pile_data.frame_id;
            object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK;
            // object.pose.covariance...  // TODO: needed?
            // object.relative_position  // ignored here
            // object.relative_yaw  // ignored here
            object.pose.pose.position.x = pile_data.position_x;
            object.pose.pose.position.y = pile_data.position_y;
            object.pose.pose.position.z = pile_data.position_z;
            float half_yaw = 0.5 * pile_data.orientation_yaw;
            object.pose.pose.orientation.x = 0.0;
            object.pose.pose.orientation.y = 0.0;
            object.pose.pose.orientation.z = sin(half_yaw);
            object.pose.pose.orientation.w = cos(half_yaw);
            object.scale.x = pile_data.scale_x;
            object.scale.y = pile_data.scale_y;
            object.scale.z = pile_data.scale_z;
            object.color = color_from_string(pile_data.color);
            targets_.objects.push_back(object);
        }
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_node");

    double frequency;
    std::string estimator_name;
    std::string robot_ns;
    ros::param::param<double>("~frequency", frequency, 1.0);
    ros::param::param<std::string>("~estimator", estimator_name, "a_priori_info");
    ros::param::param<std::string>("~robot_ns", robot_ns, "mbzirc2020");

    Estimator* estimator = nullptr;
    if (estimator_name == "simple_clustering") {
        estimator = new SimpleClusteringEstimator(frequency, robot_ns);
    } else if (estimator_name == "a_priori_info") {
        estimator = new APrioriInfoEstimator(frequency);
    } else {
        ROS_WARN("Unexpected estimator name [%s], using default", estimator_name.c_str());
        estimator = new APrioriInfoEstimator(frequency);
    }

    ros::spin();

    return 0;
}