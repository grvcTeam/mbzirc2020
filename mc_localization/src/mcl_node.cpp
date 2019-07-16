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
    Pose2D() = default;
    Pose2D(double _x, double _y, double _yaw) {
        x = _x;
        y = _y;
        yaw = _yaw;
    }
    double x;
    double y;
    double yaw;
};

struct Noise {
    std::normal_distribution<double> move  = std::normal_distribution<double>(0, 0.25);
    std::normal_distribution<double> turn  = std::normal_distribution<double>(0, 0.25);
    std::normal_distribution<double> sense = std::normal_distribution<double>(0, 1.00);
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
    double angle_inc_ =  0.1;     // [rad]
    double range_min_ =  0.1;       // [m]
    double range_max_ = 10.0;       // [m]
};

// TODO: Wouldn't be faster to just do one conversion, from real measurement in meters to pixels,
// instead of many conversions from pixel to meters (enhancement)

class ParticleSet {
public:
    ParticleSet(const std::string& _id, const std::string& _laser_sub_topic = "scan", const std::string& _map_topic = "map") {
        id_ = _id;
        laser_pub_topic_ = id_ + "/scan";
        laser_sub_topic_ = _laser_sub_topic;
        map_topic_ = _map_topic;
    
        ros::NodeHandle n;
        laser_pub_ = n.advertise<sensor_msgs::LaserScan>(laser_pub_topic_, 1);
        map_sub_ = n.subscribe(map_topic_, 1, &ParticleSet::mapCallback, this);
        laser_sub_ = n.subscribe(laser_sub_topic_, 1, &ParticleSet::laserCallback, this);
        ray_cast_ = std::make_shared<RayCast>();
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

    void setNoise(const Noise& _noise) {
        // TODO: assert correct values
        noise_ = _noise;
    }

    bool is_ready() {
        if (!scan_updated_) { 
            ROS_ERROR("Scan not updated!"); 
            return false;
        }
        if (!map_loaded_) { 
            ROS_ERROR("Map not loaded!"); 
            return false;
        }
        if (poses_.size() == 0) {
            ROS_ERROR("Poses set is empty!");
            return false;
        }
        // if (weights_.size() == 0) {
        //     ROS_ERROR("Weights set is empty!");
        //     return false;
        // }
        return true;
    }

    void update(const Pose2D& _delta_pose) {
        if (!is_ready()) { return; }
        move(_delta_pose);
        weigh();
        resample();
    }

    void print() {
        if (!is_ready()) { return; }
        for(size_t i = 0; i < poses_.size(); i++) {
            ROS_INFO("[%ld] in [%lf, %lf, %lf] with weight [%lf]", i, poses_[i].x, poses_[i].y, poses_[i].yaw, weights_[i]);
        }
    }

    Pose2D mean() {
        return mean_pose_;
    }

    Pose2D best() {
        if (!is_ready()) {
            return Pose2D(0, 0, 0);  // Default pose
        }
        auto max_i = std::distance(weights_.begin(), std::max_element(weights_.begin(), weights_.end()));
        return poses_[max_i];
    }

    void publish_sim(const Pose2D& _laser_pose) {
        if (!map_loaded_) {
            ROS_WARN("publish_sim: No map yet!");
        }

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped tf_laser;
        
        tf_laser.header.stamp = ros::Time::now();
        tf_laser.header.frame_id = "map";
        tf_laser.child_frame_id = id_;
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
        scan.header.frame_id = id_;
        laser_pub_.publish(scan);
    }

protected:

    void mapCallback(const nav_msgs::OccupancyGrid::Ptr& msg) {
        ROS_INFO("ParticleSet: Updating map...");
        map_ = *msg;
        cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width, CV_8UC1, map_.data.data());  // Convert OccupancyGrid to cv::Mat, uint8_t
        cv::threshold(map_mat, map_mat, 254, 255, cv::THRESH_TOZERO_INV);  // Set unknown space (255) to free space (0)
        ray_cast_->setGrid(map_mat, map_.info.resolution, cv::Point(map_.info.origin.position.x, map_.info.origin.position.y));  // Update map
        map_x_min_ = map_.info.origin.position.x;
        map_x_max_ = map_x_min_ + map_.info.width * map_.info.resolution;
        map_y_min_ = map_.info.origin.position.y;
        map_y_max_ = map_y_min_ + map_.info.height * map_.info.resolution;
        map_loaded_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_ = *msg;
        scan_updated_ = true;
    }

    bool poseIsInMap(const Pose2D& _pose) {
        if (_pose.x < map_x_min_ || _pose.x > map_x_max_) {
            ROS_ERROR("poseIsInMap: %lf out of x bounds", _pose.x);
            return false;
        }
        if (_pose.y < map_y_min_ || _pose.y > map_y_max_) {
            ROS_ERROR("poseIsInMap: %lf out of y bounds", _pose.y);
            return false;
        }
        return true;
    }

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
            if (poseIsInMap(new_pose)) {
                poses_[i] = new_pose;
            }
        }
    }

    void weigh() {
        double total_weight = 0;
        ray_cast_->setScanParams(scan_);
        double sense_std_dev = noise_.sense.stddev();
        for (size_t i = 0; i < poses_.size(); i++) {
            std::vector<float> sensed = ray_cast_->scan(poses_[i]);  // TODO: there is no sense_noise here?
            if (scan_.ranges.size() != sensed.size()) {
                ROS_ERROR("Scan size (%ld) differs from sensed size (%ld)", scan_.ranges.size(), sensed.size());
            }
            // TODO: Use OpenCV to compare scans?
            double w = 1.0;
            for (size_t j = 0; j < scan_.ranges.size(); j++) {
                w *= normal_pdf(sensed[j], sense_std_dev, scan_.ranges[j]);
            }
            weights_[i] = w;
            total_weight += w;
        }
        mean_pose_ = Pose2D(0, 0, 0);
        for (size_t i = 0; i < weights_.size(); i++) {
            weights_[i] /= total_weight;
            mean_pose_.x += weights_[i] * poses_[i].x;
            mean_pose_.y += weights_[i] * poses_[i].y;
            mean_pose_.yaw += weights_[i] * poses_[i].yaw;
        }
    }

    void resample() {
        int n = poses_.size();

        std::uniform_int_distribution<int> uniform_int(0, n);
        int index = uniform_int(random_engine_);
        double beta = 0;
        double max_weight = *std::max_element(weights_.begin(), weights_.end());
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

    std::string id_;
    std::vector<Pose2D> poses_;
    std::vector<double> weights_;
    Pose2D mean_pose_ = Pose2D(0, 0, 0);

    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LaserScan scan_;
    bool map_loaded_ = false;
    bool scan_updated_ = false;
    std::shared_ptr<RayCast> ray_cast_;
    double map_x_min_;
    double map_x_max_;
    double map_y_min_;
    double map_y_max_;

    Noise noise_;
    std::default_random_engine random_engine_;
    // std::random_device rd;
    // std::mt19937 random_engine_(rd());

    ros::Publisher laser_pub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber map_sub_;

    std::string laser_pub_topic_;
    std::string laser_sub_topic_;
    std::string map_topic_;
};


int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "mcl_node");

    ParticleSet laser("laser");
    ParticleSet particle_set("pf", "laser/scan");

    Pose2D pose(1, 2, 0.5);

    std::uniform_real_distribution<double> x_position(-1.0, 3.0);
    std::uniform_real_distribution<double> y_position(-0.0, 4.0);
    std::uniform_real_distribution<double> yaw_angle(0.0, 1.0);
    particle_set.init(1000, x_position, y_position, yaw_angle);

    ros::Rate rate(10);  // [Hz]
    while(ros::ok()) {
        ros::spinOnce();
        laser.publish_sim(pose);
        particle_set.update(Pose2D(0,0,0));  // Still!
        // particle_set.print();
        particle_set.publish_sim(particle_set.mean());
        rate.sleep();
    }

    return 0;
}
