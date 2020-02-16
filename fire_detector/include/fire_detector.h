//----------------------------------------------------------------------------------------------------------------------
// Fire Detector
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef FIRE_DETECTOR_THERMAL_H
#define FIRE_DETECTOR_THERMAL_H

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Temperature.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/CheckFire.h>

#define CONV2PNT 4 // Laser angle amplitude: 1 grade = 4 point
#define M_TEMP  32 // Size of Temp Matrix
#define MIN_TEMP_DEFAULT 1000
#define LASER_RANGE 720 // Lidar - full range of points
#define SCALE_FACTOR 20 // To improve debug view
#define R_CIRCLE 70.0 // Radius of the circle showed in debug view
#define MAX_FILTER_NEGATIVES 30 // Num of needed continue iterations to consider a negative measure as false (15Hz*2s=30frames)
#define MIN_FILTER_POSITIVES 15 // Num of needed continue iterations to consider a positive measure as true (15Hz*1s=15frames)

class Thermal {
public:
    Thermal();

protected:
    void thermalDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ualPoseCallback(const geometry_msgs::PoseStamped& msg);
    void laserCallback(const sensor_msgs::LaserScan& msg);
    bool checkFireCallback(mbzirc_comm_objs::CheckFire::Request& req, mbzirc_comm_objs::CheckFire::Response& res);

    ros::Publisher pub_detect_img_;
    ros::Publisher pub_sensed_;
    ros::Publisher pub_debug_;
    ros::Subscriber sub_raw_temp_;
    ros::Subscriber sub_rgb_img_;
    ros::Subscriber sub_ual_pose_;
    ros::Subscriber sub_scan_;
    ros::ServiceServer srv_checkfire_;

    std::string uav_id_;
    geometry_msgs::PointStamped uav_position_;
    float uav_yaw_;
    std::string mode_;
    float temp_matrix_[M_TEMP][M_TEMP];
    float max_temp_;
    int thermal_threshold_;
    float sigma_[3];  // xyz  
    bool debug_publisher_;
    bool debug_view_;
    int angle_amplitude_;
    int initial_index_;
    float laser_measurement_;
    short int false_negative_;
    short int false_positive_;
    bool detected_;
};

#endif  // FIRE_DETECTOR_THERMAL_H
