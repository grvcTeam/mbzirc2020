#include <random>
#include <algorithm>
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

struct Pose2D {
    double x;
    double y;
    double yaw;
};

struct Noise {
    std::normal_distribution<double> move  = std::normal_distribution<double>(0, 0.2);
    std::normal_distribution<double> turn  = std::normal_distribution<double>(0, 0.2);
    std::normal_distribution<double> sense = std::normal_distribution<double>(0, 2.0);
};

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

inline bool poseIsInMap(const Pose2D& _pose, const nav_msgs::MapMetaData& _map_info) {
    // TODO: Calculate this each time is not efficient!
    double x_min = _map_info.origin.position.x;
    double x_max = x_min + _map_info.width * _map_info.resolution;
    double y_min = _map_info.origin.position.y;
    double y_max = y_min + _map_info.height * _map_info.resolution;
    ROS_INFO("map: [%lf, %lf] x [%lf, %lf]", x_min, x_max, y_min, y_max);

    if (_pose.x < x_min || _pose.x >= x_max) {
        ROS_ERROR("poseIsInMap: %lf out of x bounds", _pose.x);
        return false;
    }
    if (_pose.y < y_min || _pose.y >= y_max) {
        ROS_ERROR("poseIsInMap: %lf out of y bounds", _pose.y);
        return false;
    }
    return true;
}

class RayCast {
public:

    RayCast() = default;

    void setScanParams(const sensor_msgs::LaserScan& _scan_params) {
        angle_min_ = _scan_params.angle_min;
        angle_max_ = _scan_params.angle_max;
        angle_inc_ = _scan_params.angle_increment;
        range_min_ = _scan_params.range_min;
        range_max_ = _scan_params.range_max;
    }

    sensor_msgs::LaserScan getScanParams() {
        sensor_msgs::LaserScan scan_params;
        scan_params.angle_min       = angle_min_;
        scan_params.angle_max       = angle_max_;
        scan_params.angle_increment = angle_inc_;
        scan_params.range_min       = range_min_;
        scan_params.range_max       = range_max_;

        return scan_params;
    }

    void setGrid(cv::Mat& _grid, double _m_per_px, cv::Point _offset) {
        grid_ = _grid;
        m_per_px_ = _m_per_px;
        grid_offset_ = _offset;
    }

    std::vector<float> scan(Pose2D _laser_pose) {
        std::vector<float> ranges;
        cv::Point2f start((_laser_pose.x - grid_offset_.x) / m_per_px_, (_laser_pose.y - grid_offset_.y) / m_per_px_);
        double yaw = _laser_pose.yaw;

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
                ranges.push_back(range);  // TODO: Add noise here?
            } else {
                ranges.push_back(range_max_ + 1.0);
            }
        }
        return ranges;
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

    LaserSim() {
        ray_cast_ = std::make_shared<RayCast>();
        // Subscribe / Publish
        ros::NodeHandle node;
        map_sub_ = node.subscribe(map_topic_, 1, &LaserSim::mapCallback, this);
        laser_pub_ = node.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::Ptr& grid) {
        ROS_INFO("LaserSim: Updating map...");
        map_ = *grid;

        cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width, CV_8UC1, map_.data.data());  // Convert OccupancyGrid to cv::Mat, uint8_t
        cv::threshold(map_mat, map_mat, 254, 255, cv::THRESH_TOZERO_INV);  // Set unknown space (255) to free space (0)
        ray_cast_->setGrid(map_mat, map_.info.resolution, cv::Point(map_.info.origin.position.x, map_.info.origin.position.y));  // Update map
        map_loaded_ = true;
    }

    void publish_sim(const Pose2D& _laser_pose) {
        if (!map_loaded_) {
            ROS_WARN("LaserSim: Update called, no map yet");
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

        sensor_msgs::LaserScan scan = ray_cast_->getScanParams();
        scan.ranges = ray_cast_->scan(_laser_pose);
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

/*
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1, 2);
    for (int n = 0; n < 10; ++n) {
        std::cout << dis(gen) << ' ';
    }
    std::cout << '\n';
 */
/*
    std::random_device rd;
    std::mt19937 gen(rd());
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(5,2);
 
    std::map<int, int> hist;
    for(int n=0; n<10000; ++n) {
        ++hist[std::round(d(gen))];
    }
    for(auto p : hist) {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
 */

class ParticleSet {
public:
    std::vector<Pose2D> poses_;
    std::vector<double> weights_;

    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LaserScan scan_;
    bool map_loaded_ = false;
    bool scan_updated_ = false;
    std::shared_ptr<RayCast> ray_cast_;

    Noise noise_;
    std::default_random_engine random_engine_;
    // std::random_device rd;
    // std::mt19937 gen(rd());

    // ros::Publisher laser_pub_;
    ros::Subscriber map_sub_;
    ros::Subscriber laser_sub_;

    std::string map_topic_ = "/map";
    // std::string laser_topic_ = "/scan";
    std::string laser_sub_topic_ = "/scan";

    ParticleSet() {

        ray_cast_ = std::make_shared<RayCast>();

        // Subscribe / Publish
        ros::NodeHandle n;
        // laser_pub_ = n.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
        map_sub_ = n.subscribe(map_topic_, 1, &ParticleSet::mapCallback, this);
        laser_sub_ = n.subscribe(laser_sub_topic_, 1, &ParticleSet::laserCallback, this);
    }

    void init(size_t _particle_count, std::uniform_real_distribution<double>& _x, std::uniform_real_distribution<double>& _y, std::uniform_real_distribution<double>& _yaw) {
        poses_.resize(_particle_count);
        weights_.resize(_particle_count);
        for (size_t i = 0; i < poses_.size(); i++) {
            poses_[i].x   = _x(random_engine_);
            poses_[i].y   = _y(random_engine_);
            poses_[i].yaw = _yaw(random_engine_);
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::Ptr& msg) {
        ROS_INFO("ParticleSet: Updating map...");
        map_ = *msg;

        cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width, CV_8UC1, map_.data.data());  // Convert OccupancyGrid to cv::Mat, uint8_t
        cv::threshold(map_mat, map_mat, 254, 255, cv::THRESH_TOZERO_INV);  // Set unknown space (255) to free space (0)
        ray_cast_->setGrid(map_mat, map_.info.resolution, cv::Point(map_.info.origin.position.x, map_.info.origin.position.y));  // Update map
        map_loaded_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_ = *msg;
        scan_updated_ = true;
    }

    void setNoise(const Noise& _noise) {
        // TODO: assert correct values
        noise_ = _noise;
    }

    // void setPose(const Pose2D& _pose) {
    //     if (!poseIsInMap(_pose, map_)) {
    //         ROS_ERROR("setPose: pose is not in map!");
    //         return;
    //     }

    //     pose_ = _pose;

    //     if (pose_.yaw < -M_PI || pose_.yaw > M_PI) {
    //         ROS_WARN("setPose: %lf out of yaw bounds, normalizing...", pose_.yaw);
    //         pose_.yaw = normalizeAngle(pose_.yaw);
    //     }
    // }

    void move(const Pose2D& _delta_pose) {
        // TODO: Make noise for all particles equal?
        for (size_t i = 0; i < poses_.size(); i++) {
            Pose2D new_pose = poses_[i];
            // Turn, and add randomness to the turning command
            new_pose.yaw += _delta_pose.yaw + noise_.turn(random_engine_);
            new_pose.yaw = normalizeAngle(new_pose.yaw);

            // Move, and add randomness to the motion command
            new_pose.x += _delta_pose.x + noise_.move(random_engine_);  // TODO: noise is added twice!
            new_pose.y += _delta_pose.y + noise_.move(random_engine_);  // TODO: noise is added twice!
            if (poseIsInMap(new_pose, map_.info)) {
                poses_[i] = new_pose;
            }
        }
    }

    void weigh() {
        if (!scan_updated_) { 
            ROS_ERROR("weigh: scan not updated!"); 
            return;
        }
        if (!map_loaded_) { 
            ROS_ERROR("weigh: map not loaded!"); 
            return;
        }

        ray_cast_->setScanParams(scan_);
        double sense_std_dev = noise_.sense.stddev();
        printf("sense_std_dev = %lf\n", sense_std_dev);
        for (size_t i = 0; i < poses_.size(); i++) {
            std::vector<float> sensed = ray_cast_->scan(poses_[i]);  // TODO: there is no sense_noise here?
            if (scan_.ranges.size() != sensed.size()) {
                ROS_ERROR("Scan size (%ld) differs from sensed size (%ld)", scan_.ranges.size(), sensed.size());
            }
            // TODO: Use OpenCV to compare scans?
            double w = 1.0;
            for (size_t j = 0; j < scan_.ranges.size(); j++) {
                // printf("normal_pdf(sensed[j], sense_std_dev, scan_.ranges[j]) = %lf\n", normal_pdf(sensed[j], sense_std_dev, scan_.ranges[j]));
                printf("w at iteration [%ld] = %lf\n", j, w);
                w *= normal_pdf(sensed[j], sense_std_dev, scan_.ranges[j]);
            }
            weights_[i] = w;
            printf("weight[%ld] = %lf\n", i, w);
        }
    }

    void resample() {
        int n = poses_.size();
        if (n == 0) {
            ROS_ERROR("Trying to resample an empty set!");
            return;
        }

        std::uniform_int_distribution<int> uniform_int(0, n);
        int index = uniform_int(random_engine_);
        double beta = 0;
        double max_weight = *std::max_element(weights_.begin(), weights_.end());
        ROS_INFO("Max weight = %lf", max_weight);
        std::uniform_real_distribution<double> uniform_real(0.0, 2.0 * max_weight);

        std::vector<Pose2D> new_set(n);
        for (int i = 0; i < n; i++) {
            beta += uniform_real(random_engine_);
            while (beta > weights_[index]) {
                beta -= weights_[index];
                index = (index + 1) % n;
            }
            new_set[i] = poses_[index];
        }
        poses_ = new_set;
    }
    // TODO: move + weigh + resample = update (only public interface!) (all checks: n, map, scan)

    void print() {
        if (!scan_updated_) { 
            ROS_ERROR("print: scan not updated!"); 
            return;
        }
        if (!map_loaded_) { 
            ROS_ERROR("print: map not loaded!"); 
            return;
        }

        for(size_t i = 0; i < poses_.size(); i++) {
            ROS_INFO("[%ld] in [%lf, %lf, %lf] with weight [%lf]", i, poses_[i].x, poses_[i].y, poses_[i].yaw, weights_[i]);
        }
    }
};


int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "mcl_node");

    LaserSim laser;
    ParticleSet particle_set;

    Pose2D pose;
    pose.x = 1;
    pose.y = 2;
    pose.yaw = 0.5;

    std::uniform_real_distribution<double> x_position(-10.0, 10.0);
    std::uniform_real_distribution<double> y_position(-10.0, 10.0);
    std::uniform_real_distribution<double> yaw_angle(-M_PI, M_PI);
    particle_set.init(10, x_position, y_position, yaw_angle);

    ros::Rate rate(0.5);  // [Hz]
    while(ros::ok()) {
        ros::spinOnce();
        laser.publish_sim(pose);
        particle_set.weigh();
        particle_set.resample();
        particle_set.print();
        rate.sleep();
    }

    return 0;
}

