#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <random_numbers/random_numbers.h>

struct LineModel {
    // a*x + b*y + c = 0
    float a;
    float b;
    float c;
};

struct RansacOutput {
    float score;
    LineModel model;
    std::vector<int> inliers;
    geometry_msgs::Point p;
    geometry_msgs::Point q;
};

RansacOutput ransac(const std::vector<geometry_msgs::Point>& _points, random_numbers::RandomNumberGenerator *_random) {
    int max_iterations = 100;  // TODO: as a function of _points.size()?
    float error_th = 0.1;
    
    RansacOutput best;
    best.score = std::numeric_limits<float>::max();
    
    for (int i = 0; i < max_iterations; i++) {
        int random_1 = _random->uniformInteger(0, _points.size());
        int random_2 = _random->uniformInteger(0, _points.size() - 1);  // Sample from smaller interval...
        if (random_2 >= random_1) { random_2 += 1; }  // ...now correct and assure random_2 != random_1
        // ROS_INFO("[%d, %d]", random_1, random_2);
        geometry_msgs::Point p_1 = _points[random_1];
        geometry_msgs::Point p_2 = _points[random_2];

        LineModel current_model;
        current_model.a = p_2.y - p_1.y;
        current_model.b = p_1.x - p_2.x;
        current_model.c = p_2.x*p_1.y - p_1.x*p_2.y;

        float current_score = 0;
        std::vector<int> current_inliers;
        for (int j = 0; j < _points.size(); j++) {
            float current_error = fabs(current_model.a*_points[j].x + current_model.b*_points[j].y + current_model.c) / sqrt(current_model.a*current_model.a + current_model.b*current_model.b);
            if (current_error < error_th) {
                current_score += current_error;
                current_inliers.push_back(j);
            } else {
                current_score += error_th;
            }
        }

        if (current_score < best.score ) {
            best.score = current_score;
            best.model = current_model;
            best.inliers = current_inliers;
            best.p = p_1;
            best.q = p_2;
        }    
    }

    return best;
}

visualization_msgs::Marker getPointsMarker(const std::vector<geometry_msgs::Point>& _points, const std::string& _frame_id, std_msgs::ColorRGBA _color, unsigned int _id = 0) {
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = _frame_id;
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "points";
    points_marker.id = _id;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.position.x = 0;
    points_marker.pose.position.y = 0;
    points_marker.pose.position.z = 0;
    points_marker.pose.orientation.x = 0.0;
    points_marker.pose.orientation.y = 0.0;
    points_marker.pose.orientation.z = 0.0;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.1;
    points_marker.scale.y = 0.1;
    //points_marker.scale.z = 0.1;
    for (auto p : _points) {
        points_marker.points.push_back(p);
    }
    points_marker.color = _color;
    points_marker.lifetime = ros::Duration();

    return points_marker;
}

// TODO: segment!
visualization_msgs::Marker getLineMarker(const RansacOutput& _result, const std::string& _frame_id, std_msgs::ColorRGBA _color, unsigned int _id = 0) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = _frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "ransac_line";
    line_marker.id = _id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.pose.orientation.x = 0.0;
    line_marker.pose.orientation.y = 0.0;
    line_marker.pose.orientation.z = 0.0;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;
    line_marker.scale.y = 0.1;
    //line_marker.scale.z = 0.1;
    float v_x = _result.q.x - _result.p.x;
    float v_y = _result.q.y - _result.p.y;
    float l = 10.0/sqrt(v_x*v_x + v_y*v_y);
    float angle = atan2(v_y, v_x);
    // ROS_INFO("angle = %f", angle);
    geometry_msgs::Point p_a, p_b;
    p_a.x = _result.p.x + (0.5 + l) * v_x;
    p_b.x = _result.p.x + (0.5 - l) * v_x;
    p_a.y = _result.p.y + (0.5 + l) * v_y;
    p_b.y = _result.p.y + (0.5 - l) * v_y;
    p_a.z = 0.5 * (_result.p.z + _result.q.z);
    p_b.z = 0.5 * (_result.p.z + _result.q.z);
    line_marker.points.push_back(p_a);
    line_marker.points.push_back(p_b);
    line_marker.color = _color;
    line_marker.lifetime = ros::Duration();

    return line_marker;
}

class PassageDetectionNode {
public:
    PassageDetectionNode() {
        marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
        scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 1, &PassageDetectionNode::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& _msg) {
        // ros::Time ransac_start = ros::Time::now();
        // RansacOutput result = ransac(points, &random);
        // ros::Duration ransac_elapsed = ros::Time::now() - ransac_start;
        // ROS_INFO("elapsed: %f", ransac_elapsed.toSec());
        // ROS_INFO("best_model = [%f, %f, %f]", result.model.a, result.model.b, result.model.c);

        // TODO: Use laser_geometry package?
        std::vector<geometry_msgs::Point> points;
        for (int i = 0; i < _msg->ranges.size(); i++) {
            if (_msg->ranges[i] < _msg->range_min) { continue; }
            if (_msg->ranges[i] > _msg->range_max) { continue; }

            float angle = _msg->angle_min + i * _msg->angle_increment;  // TODO: use angle_max?
            // TODO: Use scan_time + time_increment?
            geometry_msgs::Point point;
            point.x = _msg->ranges[i] * cos(angle);
            point.y = _msg->ranges[i] * sin(angle);
            points.push_back(point);
        }

        RansacOutput line = ransac(points, &random_);

        visualization_msgs::MarkerArray marker_array;
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 0.5;
        marker_array.markers.push_back(getPointsMarker(points, _msg->header.frame_id, color));
        marker_array.markers.push_back(getLineMarker(line, _msg->header.frame_id, color));

        marker_pub_.publish(marker_array);
    }

protected:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Subscriber scan_sub_;
    random_numbers::RandomNumberGenerator random_;
};


int main(int _argc, char **_argv) {

    ros::init(_argc, _argv, "passage_detection_node");

    PassageDetectionNode passage_detection_node;

    ros::spin();

    return 0;
}
