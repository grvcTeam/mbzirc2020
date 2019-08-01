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
            for (auto target: targets_.objects) {
                if (squared_distance(sensed.pose.pose.position, target.pose.pose.position) < squared_distance_th_) {
                    first_time_sensed = false;
                    break;
                }
            }
            if (first_time_sensed) {
                // TODO: id? Use index?
                targets_.objects.push_back(sensed);
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
    float squared_distance_th_ = 25.0;
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_node");

    // TODO: parameters from rosparam
    DummyEstimator estimation("mbzirc2020", 2);
    ros::spin();

    return 0;
}
