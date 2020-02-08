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
#include <opencv2/opencv.hpp>

#define CONV2PNT 4 // Laser angle amplitude: 1 grade = 4 point
#define M_TEMP  32 // Size of Temp Matrix
#define MIN_TEMP_DEFAULT 1000
#define LASER_RANGE 720 // Lidar - full range of points
#define SCALE_FACTOR 20 // To improve debug view
#define R_CIRCLE 70.0 // Radius of the circle showed in debug view

class Thermal {
public:
    Thermal();
    void thermal_data(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void ual_to_fire_position(const geometry_msgs::PoseStamped& msg);
    void laser_measures(const sensor_msgs::LaserScan& msg);
    void image_operations(const sensor_msgs::ImageConstPtr& msg);

protected:
    geometry_msgs::PointStamped uav_position;
    ros::Publisher pub;
    ros::Publisher pub_msg;
    float max_temp;
    int angle_amplitude;
    int initial;
    float temp_matrix[M_TEMP][M_TEMP];
    float uav_yaw;
    float laser_measurement;
    std::string uav_id;
};

#endif  // FIRE_DETECTOR_THERMAL_H
