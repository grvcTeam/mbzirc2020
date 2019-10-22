#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
// #include <mbzirc_comm_objs/DetectTypes.h>
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
    std::vector<geometry_msgs::Point> inliers;
    std::vector<geometry_msgs::Point> outliers;
    geometry_msgs::Point p;
    geometry_msgs::Point q;
};

RansacOutput ransac(const std::vector<geometry_msgs::Point>& _points, random_numbers::RandomNumberGenerator *_random) {
    if (_points.size() < 2) {
        // ROS_ERROR("ransac: _points.size() < 2");
        return RansacOutput();
    }

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
        float current_error_denominator = sqrt(current_model.a*current_model.a + current_model.b*current_model.b);

        float current_score = 0;
        std::vector<geometry_msgs::Point> current_inliers;
        std::vector<geometry_msgs::Point> current_outliers;
        for (int j = 0; j < _points.size(); j++) {
            float current_error = fabs(current_model.a*_points[j].x + current_model.b*_points[j].y + current_model.c) / current_error_denominator;
            if (current_error < error_th) {
                current_score += current_error;
                current_inliers.push_back(_points[j]);
            } else {
                current_score += error_th;
                current_outliers.push_back(_points[j]);
            }
        }

        if (current_score < best.score ) {
            best.score = current_score;
            best.model = current_model;
            best.inliers = current_inliers;
            best.outliers = current_outliers;
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
    points_marker.lifetime = ros::Duration(0.1);

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
    line_marker.points.push_back(_result.p);
    line_marker.points.push_back(_result.q);
    line_marker.color = _color;
    line_marker.lifetime = ros::Duration(0.1);

    return line_marker;
}

// TODO: More colors than just r/g/b
std_msgs::ColorRGBA colorFromIndex(int _index) {
    std_msgs::ColorRGBA color;
    switch (_index % 3) {
        case 0:
            color.r = 1.0;
            break;
        case 1:
            color.g = 1.0;
            break;
        case 2:
            color.b = 1.0;
            break;        
    }
    color.a = 0.5;

    return color;
}

class PassageDetectionNode {
public:
    PassageDetectionNode() {
        marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
        sensed_pub_ = n_.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 3);
        scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 1, &PassageDetectionNode::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& _msg) {
        size_t max_line_count = 4;  // TODO: from parameter!
        size_t min_points_size = 9;  // TODO: from parameter!
        float sq_passage_th = 1.0;  // TODO: from parameter!

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
        if (points.size() < min_points_size) {
            // ROS_ERROR("scanCallback: points.size() < %d", min_points_size);
            return;
        }

        mbzirc_comm_objs::ObjectDetectionList object_list;
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < max_line_count; i++) {
            RansacOutput line = ransac(points, &random_);
            if (line.inliers.size() < 2) {
                // ROS_ERROR("scanCallback: line.inliers.size() < 2");
                continue;
            }
            std_msgs::ColorRGBA color = colorFromIndex(i);
            marker_array.markers.push_back(getPointsMarker(line.inliers, _msg->header.frame_id, color, i));
            marker_array.markers.push_back(getLineMarker(line, _msg->header.frame_id, color, i));
            for (int j = 0; j < line.inliers.size()-1; j++) {
                float delta_x = line.inliers[j+1].x - line.inliers[j].x;
                float delta_y = line.inliers[j+1].y - line.inliers[j].y;
                float delta_z = line.inliers[j+1].z - line.inliers[j].z;
                float sq_distance = delta_x*delta_x + delta_y*delta_y + delta_z*delta_z;
                // ROS_INFO("sq_distance = % f", sq_distance);
                if (sq_distance > sq_passage_th) {
                    float distance = sqrt(sq_distance);
                    mbzirc_comm_objs::ObjectDetection object;
                    object.header.frame_id = _msg->header.frame_id;
                    object.header.stamp = ros::Time::now();
                    object.type = mbzirc_comm_objs::ObjectDetection::TYPE_PASSAGE;

                    // TODO: relative_position has sense?
                    // object.relative_position.x = 
                    // object.relative_position.y = 
                    // object.relative_position.z = 
                    object.pose.pose.position.x = line.inliers[j].x + 0.5 * delta_x;
                    object.pose.pose.position.y = line.inliers[j].y + 0.5 * delta_y;
                    object.pose.pose.position.z = line.inliers[j].z + 0.5 * delta_z;

                    float theta = atan2(line.model.b, line.model.a);
                    object.relative_yaw = theta;
                    object.pose.pose.orientation.x = 0;
                    object.pose.pose.orientation.y = 0;
                    object.pose.pose.orientation.z = sin(0.5*theta);
                    object.pose.pose.orientation.w = cos(0.5*theta);
                    object.pose.covariance[0] = 0.01;  // TODO: Covariance?
                    object.pose.covariance[7] = 0.01;
                    object.pose.covariance[14] = 0.01;

                    object.scale.x = 0.1;
                    object.scale.y = distance;
                    object.scale.z = 0.1;
                    object.color = mbzirc_comm_objs::ObjectDetection::COLOR_GREEN;  // TODO!
                    object_list.objects.push_back(object);
                }
            }
            points = line.outliers;
            if (points.size() < min_points_size) { break; }
        }
        marker_pub_.publish(marker_array);  // TODO: Make visalization optional!
        sensed_pub_.publish(object_list);
    }

protected:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Publisher sensed_pub_;
    ros::Subscriber scan_sub_;
    random_numbers::RandomNumberGenerator random_;
};


int main(int _argc, char **_argv) {

    ros::init(_argc, _argv, "passage_detection_node");

    PassageDetectionNode passage_detection_node;

    ros::spin();

    return 0;
}
