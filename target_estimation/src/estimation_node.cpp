#include <ros/ros.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

float squared_distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    return dx*dx + dy*dy + dz*dz;
}

class DummyEstimator {
public:

    DummyEstimator(const std::string& _uav_ns, unsigned int _uav_count) {
        ros::NodeHandle nh;

        for (int i = 1; i < _uav_count + 1; i++) {
            std::string sensed_topic = _uav_ns + "_" + std::to_string(i) + "/sensed_objects";
            sensed_sub_.push_back(nh.subscribe(sensed_topic, 1, &DummyEstimator::updateCallback, this));
        }

        estimated_pub_ = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("estimated_objects", 1);
        estimation_timer_ = nh.createTimer(ros::Duration(1), &DummyEstimator::estimateCallback, this);  // TODO: frequency as a parameter
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

                        // float sensed_weight = sensed_radius / (sensed_radius + target_radius);
                        // float x_new = sensed_weight * sensed.pose.pose.position.x + (1.0 - sensed_weight) * target.pose.pose.position.x;
                        // float y_new = sensed_weight * sensed.pose.pose.position.y + (1.0 - sensed_weight) * target.pose.pose.position.y;
                        // float z_new = sensed_weight * sensed.pose.pose.position.z + (1.0 - sensed_weight) * target.pose.pose.position.z;

                        // float x_scale = std::max(x_max - x_new, x_new - x_min);
                        // float y_scale = std::max(y_max - y_new, y_new - y_min);
                        // float z_scale = std::max(sensed.scale.z, target.scale.z);  // Supose all objects are at thes same height

                        // TODO: Something with covariance?
                        target.pose.pose.position.x = x_new;
                        target.pose.pose.position.y = y_new;
                        target.pose.pose.position.z = z_new;
                        target.scale.x = x_scale;
                        target.scale.y = y_scale;
                        target.scale.z = z_scale;
                    }
                    // if (sensed.scale.x > target.scale.x) { target.scale.x = sensed.scale.x; }
                    // if (sensed.scale.y > target.scale.y) { target.scale.y = sensed.scale.y; }
                    // if (sensed.scale.z > target.scale.z) { target.scale.z = sensed.scale.z; }
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
                new_estimated.properties = sensed.properties;
                targets_.objects.push_back(new_estimated);
            }
        }
    }

    void estimateCallback(const ros::TimerEvent& event) {
        estimated_pub_.publish(targets_);
    }

    ros::Timer estimation_timer_;
    ros::Publisher estimated_pub_;
    std::vector<ros::Subscriber> sensed_sub_;
    mbzirc_comm_objs::ObjectDetectionList targets_;
    float squared_distance_th_ = 25.0;  // TODO: tune, from params?
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_node");

    // TODO: parameters from rosparam
    DummyEstimator estimation("mbzirc2020", 2);
    ros::spin();

    return 0;
}
