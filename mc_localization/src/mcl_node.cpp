#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// TODO: Use geometry_msgs::Pose?
struct Pose2D {
    double x;
    double y;
    double yaw;
};

struct Noise {
    std::normal_distribution<double> move  = std::normal_distribution<double>(0, 0);
    std::normal_distribution<double> turn  = std::normal_distribution<double>(0, 0);
    std::normal_distribution<double> sense = std::normal_distribution<double>(0, 0);
};

// TODO: Use OccupancyGrid
struct Map {
    double width;
    double height;
};

// TODO: Use part of LaserScan?
// struct ScanParams {};

// TODO: Use doubles or floats?
inline double normal_pdf(double mu, double sigma, double x) {
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double aux = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-0.5f * aux * aux);
} 

inline double normalizeAngle(double angle) {
    while (angle < -M_PI) angle += 2*M_PI;
    while (angle >  M_PI) angle -= 2*M_PI;
    return angle;
}

inline bool poseIsInMap(const Pose2D& _pose, const Map& _map) {
    if (_pose.x < 0 || _pose.x >= _map.width) {
        // TODO: Values < 0 are possible?
        ROS_ERROR("poseIsInMap: %lf out of x bounds", _pose.x);
        return false;
    }
    if (_pose.y < 0 || _pose.y >= _map.height) {
        // TODO: Values < 0 are possible?
        ROS_ERROR("poseIsInMap: %lf out of y bounds", _pose.y);
        return false;
    }
    return true;
}

class RayCast {
public:

    RayCast() = default;

    void setScanParams(double _angle_min, double _angle_max, double _angle_inc, double _range_min, double _range_max) {
        angle_min_ = _angle_min;
        angle_max_ = _angle_max;
        angle_inc_ = _angle_inc;
        range_min_ = _range_min;
        range_max_ = _range_max;
    }

    void setGrid(cv::Mat& _grid, double _m_per_px, cv::Point _offset) {
        grid_ = _grid;
        m_per_px_ = _m_per_px;
        grid_offset_ = _offset;
    }

    sensor_msgs::LaserScan scan(Pose2D _laser_pose) {

        cv::Point2f start((_laser_pose.x - grid_offset_.x) / m_per_px_, (_laser_pose.y - grid_offset_.y) / m_per_px_);
        double yaw = _laser_pose.yaw;

        sensor_msgs::LaserScan scan;
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_inc_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;

        cv::Point2f hit;
        double max_px = range_max_ / m_per_px_;
        for (double a = angle_min_; a <= angle_max_; a += angle_inc_) {
            cv::Point2f end = cv::Point2f(start.x + max_px * cos(yaw + a), start.y + max_px * sin(yaw + a));
            if (cast(start, end, hit)) {
                double range = cv::norm(hit - start);
                range *= m_per_px_;
                if (range < range_min_) {
                    range = range_max_ + 1.0;  // Out of range (range > range_max)
                }
                scan.ranges.push_back(range);
            } else {
                scan.ranges.push_back(range_max_ + 1.0);
            }
        }
        return scan;
    }

protected:

    bool cast(cv::Point2f& _start, cv::Point2f& _end, cv::Point2f& _hit) {
        cv::Point2i start_d = _start;
        cv::Point2i end_d   = _end;
        if (!cv::clipLine(grid_.size(), start_d, end_d)) {
            return false;
        }
        cv::LineIterator it(grid_, start_d, end_d, 8);  // line is 8-connected
        for (int i = 0; i < it.count; i++, ++it) {
            if (grid_.at<uint8_t>(it.pos()) > 0) {
                _hit = it.pos();
                return true;
            }
        }
        return false;
    }

    cv::Mat grid_;
    double m_per_px_;
    cv::Point grid_offset_;

    double angle_min_ = -M_PI_2;  // [rad]
    double angle_max_ = +M_PI_2;  // [rad]
    double angle_inc_ = 0.01;     // [rad]
    double range_min_ =  0.1;       // [m]
    double range_max_ = 10.0;       // [m]
};


class LaserSim {
public:

    LaserSim(ros::NodeHandle node) {

        ray_cast_ = std::make_shared<RayCast>();
        ray_cast_->setScanParams(node.param<double>("angle/min", -M_PI_2),
                                 node.param<double>("angle/max", M_PI_2),
                                 node.param<double>("angle/increment", 0.01),
                                 node.param<double>("range/min", 1.0),
                                 node.param<double>("range/max", 20.0)
                                 );

        // Subscribe / Publish
        map_sub_ = node.subscribe(map_topic_, 1, &LaserSim::mapCallback, this);
        laser_pub_ = node.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::Ptr& grid) {

        ROS_INFO("Updating map...");
        map_ = *grid;

        cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width, CV_8UC1, map_.data.data());  // Convert OccupancyGrid to cv::Mat, uint8_t
        cv::threshold(map_mat, map_mat, 254, 255, cv::THRESH_TOZERO_INV);  // Set unknown space (255) to free space (0)
        ray_cast_->setGrid(map_mat, map_.info.resolution, cv::Point(map_.info.origin.position.x, map_.info.origin.position.y));  // Update map
        map_loaded_ = true;
    }

    void update(Pose2D _laser_pose)  {
        if (!map_loaded_) {
            ROS_WARN("LaserSim: Update called, no map yet");
            return;
        }

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped tf_laser;
        
        tf_laser.header.stamp = ros::Time::now();
        tf_laser.header.frame_id = "map";
        tf_laser.child_frame_id = "laser";  // TODO: laser_frame_id?
        tf_laser.transform.translation.x = _laser_pose.x;
        tf_laser.transform.translation.y = _laser_pose.y;
        tf_laser.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, _laser_pose.yaw);
        tf_laser.transform.rotation.x = q.x();
        tf_laser.transform.rotation.y = q.y();
        tf_laser.transform.rotation.z = q.z();
        tf_laser.transform.rotation.w = q.w();
        tf_broadcaster.sendTransform(tf_laser);

        sensor_msgs::LaserScan scan = ray_cast_->scan(_laser_pose);
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "laser";  // TODO: laser_frame_id
        laser_pub_.publish(scan);
    }

protected:

    nav_msgs::OccupancyGrid map_;
    bool map_loaded_ = false;

    ros::Subscriber map_sub_;
    ros::Publisher laser_pub_;
    std::shared_ptr<RayCast> ray_cast_;

    std::string map_topic_ = "/map";
    std::string laser_topic_ = "/scan";
};

// TODO: Wouldn't be faster to just do one conversion, from real measurement in meters to pixels,
// instead of many conversions from pixel to meters (enhancement)

class RobotParticle {
public:
    Map map_;
    Pose2D pose_;
    Noise noise_;
    sensor_msgs::LaserScan scan_;
    std::default_random_engine random_engine_;

    RobotParticle() = default;

    void setMap(const Map& _map) {
        map_ = _map;
    }

    void setPose(const Pose2D& _pose) {
        if (!poseIsInMap(_pose, map_)) {
            ROS_ERROR("setPose: pose is not in map!");
            return;
        }

        pose_ = _pose;

        if (pose_.yaw < -M_PI || pose_.yaw > M_PI) {
            ROS_WARN("setPose: %lf out of yaw bounds, normalizing...", pose_.yaw);
            pose_.yaw = normalizeAngle(pose_.yaw);
        }
    }

    void setNoise(const Noise& _noise) {
        // TODO: assert correct values
        noise_ = _noise;
    }

    void setMeasurement(const sensor_msgs::LaserScan& _scan) {
        // TODO: this goes in a callback
        scan_ = _scan;
    }

    void sense() {
        // TODO: Here goes ray_tracing
        // Z = []
        // for (size_t i = 0; i < scan_.lenght; i++) {
        //     dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
        //     dist += random.gauss(0.0, self.sense_noise)
        //     Z.append(dist)
        // }
        // return Z
    }
    
    void move(const Pose2D& _delta_pose) {
        
        // Turn, and add randomness to the turning command
        pose_.yaw += _delta_pose.yaw + noise_.turn(random_engine_);
        pose_.yaw = normalizeAngle(pose_.yaw);

        // Move, and add randomness to the motion command
        pose_.x += _delta_pose.x + noise_.move(random_engine_);  // TODO: noise is added twice!
        pose_.y += _delta_pose.y + noise_.move(random_engine_);  // TODO: noise is added twice!
        if (!poseIsInMap(pose_, map_)) {
            ROS_ERROR("move: pose is not in map!");
            // TODO: cyclic truncate?
            return;
        }
    }
    
    double weight() {        
        double prob = 1.0;
        double sense_std_dev = noise_.sense.stddev();
        for (size_t i = 0; i < scan_.ranges.size(); i++) {
            double dist = 0;  // TODO: result from sense() function
            prob *= normal_pdf(dist, sense_std_dev, scan_.ranges[i]);
        }
        return prob;
    }

};

/*
// struct LaserConfig {

//     LaserConfig() {}
//     LaserConfig(const sensor_msgs::LaserScan& _scan) {
//         angle_min = _scan.angle_min;
//         angle_max = _scan.angle_max;
//         angle_increment = _scan.angle_increment;
//         time_increment = _scan.time_increment;
//         scan_time = _scan.scan_time;
//         range_min = _scan.range_min;
//         range_max = _scan.range_max;
//     }

//     double angle_min;        // start angle of the scan [rad]
//     double angle_max;        // end angle of the scan [rad]
//     double angle_increment;  // angular distance between measurements [rad]
//     double time_increment;   // time between measurements [seconds] - if your scanner
//                             // is moving, this will be used in interpolating position
//                             // of 3d points
//     double scan_time;        // time between scans [seconds]
//     double range_min;        // minimum range value [m]
//     double range_max;        // maximum range value [m]
// };

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO("laserCallback!");
}

sensor_msgs::LaserScan scan(nav_msgs::OccupancyGrid _map, geometry_msgs::Pose _pose, sensor_msgs::LaserScan _laser_config) {
    sensor_msgs::LaserScan output = _laser_config;  // TODO: copy just config!
    return output;
}


std::vector<sensor_msgs::LaserScan> update(const std::vector<geometry_msgs::Pose>& _poses) {
    std::vector<sensor_msgs::LaserScan> output;
    for (auto pose: _poses) {
        output.push_back(scan(map, pose, laser_config));
    }
    return output;
}
*/

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "mcl_node");
    ros::NodeHandle n;

    // ros::Subscriber laser_sub = n.subscribe("laser", 1, laserCallback);
    // RobotParticle robot;
    LaserSim laser(n);

    Pose2D pose;
    pose.x = 1;
    pose.y = 2;
    pose.yaw = 0.5;

    ros::Rate rate(10);  // [Hz]
    while(ros::ok()) {
        ros::spinOnce();
        laser.update(pose);
        rate.sleep();
    }
    
    // tf2::Transform map_to_image;
    // map_to_image.setOrigin(tf2::Vector3(-30, -10, 0));
    // map_to_image.setRotation(tf2::Quaternion(0, 0, 0, 1));
    // std::cout << tf2::toMsg(map_to_image) << '\n';

    // tf2::Transform map_to_laser;
    // map_to_laser.setOrigin(tf2::Vector3(15, 5, 0));
    // map_to_laser.setRotation(tf2::Quaternion(0, 0, 0.383, 0.924));
    // std::cout << tf2::toMsg(map_to_laser) << '\n';

    // tf2::Transform image_to_laser = map_to_image.inverseTimes(map_to_laser);
    // std::cout << tf2::toMsg(image_to_laser) << '\n';

    ros::spin();

    return 0;
}

