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
#include "ros/ros.h"
#include <fire_detector.h>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/CheckFire.h>

using namespace std;
using namespace cv;

Thermal::Thermal() {

    ros::NodeHandle n("~");
    n.getParam("uav_id", uav_id_);
    n.getParam("camera_config", mode_);
    n.getParam("thermal_threshold", thermal_threshold_);
    n.getParam("covariance_x", sigma_[0]);
    n.getParam("covariance_y", sigma_[1]);
    n.getParam("covariance_z", sigma_[2]);
    n.getParam("debug_publisher", debug_publisher_);
    n.getParam("debug_view", debug_view_);
    n.getParam("angle_amplitude", angle_amplitude_);
    n.param("num_frame_filter", num_frame_filter_, 15);
    
    angle_amplitude_ = angle_amplitude_*CONV2PNT;
    initial_index_ = LASER_RANGE/2-angle_amplitude_;  // Point to laser front
    false_negative_ = 0;
    false_positive_ = 0;
    detected_ = false;

    ros::NodeHandle nh;
    sub_raw_temp_ = nh.subscribe("teraranger_evo_thermal/raw_temp_array", 1, &Thermal::thermalDataCallback, this);
    sub_rgb_img_ = nh.subscribe("teraranger_evo_thermal/rgb_image", 1, &Thermal::thermalImageCallback, this);
    sub_ual_pose_ = nh.subscribe("ual/pose", 1, &Thermal::ualPoseCallback, this);
    sub_scan_ = nh.subscribe("scan", 1, &Thermal::laserCallback, this);

    pub_sensed_ = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1);
    if(debug_publisher_) {
        pub_debug_ = nh.advertise<sensor_msgs::Temperature>("thermal_detection/debug", 1);
    }

    srv_checkfire_ = nh.advertiseService("thermal_detection/fire_detected", &Thermal::checkFireCallback, this);
}

// Routine to find fire in the image - Just one fire in the image
// TODO - detect more than one possible fire
void Thermal::thermalDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int aux; // to loop through thermal array data
    float aux_max = 0;
    for (int j = 0, aux = 0; j < M_TEMP; j++) {
        for (int k = 0; k < M_TEMP; k++, aux++) {
            // Save current temp matrix
            temp_matrix_[k][j] = msg->data[aux];
            // Search hightest temp of the array
            if (msg->data[aux] > aux_max) {
                aux_max = msg->data[aux];
            }
        }
    }
    max_temp_ = aux_max;
}

//  Routine to obtain data pose and header to create fire messages
void Thermal::ualPoseCallback(const geometry_msgs::PoseStamped& msg) {
    // Position in x,y,z   
    uav_position_.header = msg.header;
    uav_position_.point = msg.pose.position;

	tf2::Quaternion q;
	tf2::fromMsg(msg.pose.orientation, q);
	tf2::Matrix3x3 Rot_matrix(q);
    double roll, pitch, yaw;
	Rot_matrix.getRPY(roll, pitch, yaw);
    uav_yaw_ = yaw;
}

//Routine to connect and obtain data from laser scanner and obtaining an average of the values
void Thermal::laserCallback(const sensor_msgs::LaserScan& msg) {
    float sum_measurement = 0.0;
    int n_measurement = 0;
    // Reading every measure established and calculating the average
    for (int i = 0; i < (angle_amplitude_*2); i++, n_measurement++) {
        if ((msg.ranges[initial_index_+i] > msg.range_min) && (msg.ranges[initial_index_+i] < msg.range_max)) {
            sum_measurement = msg.ranges[initial_index_+i] + sum_measurement;
        }
    }    
    laser_measurement_ = sum_measurement / n_measurement;
}

//  Routine to process the image and determine if there is fire and where
void Thermal::thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {

   // Main routine 
    try {
        // Convert msg from ros topic to image 
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "8UC3");
        cv::Mat therm = Mat::zeros(Size(M_TEMP*SCALE_FACTOR,M_TEMP*SCALE_FACTOR), CV_8UC1);  // TODO Why CV_8UC1

        // Routine to obtain a black & white filter, 
        // Black = there is no fire
        // White = pixel temperature is bigger than thermal threeshold 
        for (int i = 0; i < M_TEMP*SCALE_FACTOR; i++) {
            for (int j = 0; j < M_TEMP*SCALE_FACTOR; j++) {
                if (temp_matrix_[int(floor(i/SCALE_FACTOR))][int(floor(j/SCALE_FACTOR))] >= thermal_threshold_) {
                    *((uint8_t *)(therm.data + M_TEMP*SCALE_FACTOR * i + j)) = 255;
                }
            }
        }
        
        Point center;
        center.x = M_TEMP*SCALE_FACTOR/2.0;
        center.y = M_TEMP*SCALE_FACTOR/2.0;

        vector<vector<Point>> outline;
        findContours(therm, outline, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        vector<Moments> mu(outline.size());
        vector<Point2f> mc(outline.size());

        cv::Mat image_color;
        cvtColor(therm, image_color, CV_GRAY2BGR);
        circle(image_color, center, 1, CV_RGB(0,255,0), 1, 25, 0);

// (false_negative_ <= MAX_FILTER_NEGATIVES && temp_matrix_[int(floor(i/SCALE_FACTOR))][int(floor(j/SCALE_FACTOR))] >= (max_temp_ - 5))

        if ((false_positive_ > num_frame_filter_) && ((max_temp_ >= thermal_threshold_) || (false_negative_ <= MAX_FILTER_NEGATIVES)) && (laser_measurement_ < 5.0 || mode_=="DOWNWARD")) {
            detected_ = true;

            if (max_temp_ <= thermal_threshold_) {
                false_negative_++;
            } else {
                false_negative_ = 0;
            }

            float cx[SCALE_FACTOR], cy[SCALE_FACTOR];
            // Calculating moments and centers in the fire
            for (int d = 0; d < outline.size(); d++) {
                // Obtaining centers in the fire
                mu[d] = moments(outline[d], false);
                mc[d] = Point2f(mu[d].m10/mu[d].m00, mu[d].m01/mu[d].m00);
                cx[d] = mu[d].m10/mu[d].m00;
                cy[d] = mu[d].m01/mu[d].m00;
                // Painting a rectangle in the fire
                Rect rect = boundingRect(outline[d]);
                circle(image_color, mc[d], 2, CV_RGB(0,255,0), 1, 16, 0);
            }

            Point sum;
            float x_comp, y_comp;
            for (int d = 0; d < outline.size(); d++) {
                sum.x = cx[d] + sum.x;
                sum.y = cy[d] + sum.y;
                if (d == outline.size() - 1) {
                    sum.y = sum.y / outline.size();
                    sum.x = sum.x / outline.size();
                    circle(image_color, sum, R_CIRCLE, CV_RGB(0,255,0), 1, 16, 0);
                    circle(image_color, sum, 3, CV_RGB(0,255,0), 1, 16, 0);
                    line(image_color, center, sum, CV_RGB(0,255,0), 1, 16, 0);

                    x_comp = center.y - sum.y;
                    y_comp = sum.x - center.x;  // TODO: Check
                }
            }

            mbzirc_comm_objs::ObjectDetection rec_object;
            rec_object.header.stamp = ros::Time::now();
            rec_object.header.frame_id = uav_position_.header.frame_id;
            rec_object.type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE;
            rec_object.color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
            rec_object.pose.covariance = { sigma_[0]*sigma_[0], 0, 0, 0, 0, 0,
                                           0, sigma_[1]*sigma_[1], 0, 0, 0, 0,
                                           0, 0, sigma_[2]*sigma_[2], 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0};
            rec_object.image_detection.img_height = M_TEMP;
            rec_object.image_detection.img_width = M_TEMP;
            rec_object.image_detection.v = y_comp/SCALE_FACTOR;
            rec_object.image_detection.u = x_comp/SCALE_FACTOR;
            rec_object.image_detection.height = R_CIRCLE/SCALE_FACTOR;
            rec_object.image_detection.width = R_CIRCLE/SCALE_FACTOR;

            if (mode_ == "DOWNWARD") {
                //Thermal image fields
                rec_object.image_detection.depth = uav_position_.point.z;
                rec_object.image_detection.camera_direction = mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_DOWNWARD;
                // Object Detection fields
                rec_object.pose.pose.position.x = uav_position_.point.x;
                rec_object.pose.pose.position.y = uav_position_.point.y;
                rec_object.pose.pose.position.z = 0.0;
            } else if (mode_=="FORWARD") {
                //Thermal image fields
                rec_object.image_detection.depth = laser_measurement_;
                rec_object.image_detection.camera_direction = mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_FORWARD;
                //Object detection fields
                rec_object.pose.pose.position.x = uav_position_.point.x + laser_measurement_*cos(uav_yaw_);
                rec_object.pose.pose.position.y = uav_position_.point.y + laser_measurement_*sin(uav_yaw_);
                rec_object.pose.pose.position.z = uav_position_.point.z;
            }

            if (debug_view_) {
                if (false_negative_) {
                    cout << "Possible false negative: Max temp detected-> " << max_temp_ << " | Current threshold-> " << thermal_threshold_ << endl;
                } else {
                    // Fire detected
                    cout << "Fire_detected" << endl;
                    cout << "Distance to center ~ x: " << to_string(x_comp) << "  y:" << to_string(y_comp) << " total:" << to_string(sqrt(x_comp*x_comp+y_comp*y_comp)) << endl;
                }
            }

            // Publishing the Object    
            mbzirc_comm_objs::ObjectDetectionList rec_list;
            rec_list.objects.push_back(rec_object); // TODO - Check if publish full list or current point
            rec_list.stamp = ros::Time::now();
            rec_list.agent_id = uav_id_;
            pub_sensed_.publish(rec_list);
            rec_list.objects.clear();
        } else {

            detected_ = false;
            if ((max_temp_ >= thermal_threshold_) && (laser_measurement_ < 5.0 || mode_=="DOWNWARD")) {
                false_positive_++;
                if (debug_view_) {
                    cout << "POSSIBLE FIRE DETECTED: Max temp detected-> " << max_temp_ << " | Current threshold-> " << thermal_threshold_ << endl;
                }
            } else {
                false_positive_ = 0;
                if (debug_view_) {
                    cout << "FIRE NOT DETECTED: Max temp detected-> " << max_temp_ << " | Current threshold-> " << thermal_threshold_ << endl;
                }
            }
        }

        flip(image_color, image_color, -1);
        rotate(image_color, image_color, ROTATE_90_COUNTERCLOCKWISE);
        
        if (debug_view_) {
            // Displaying images on screen
            string thermal_image_window = "thermal_image";
            namedWindow(thermal_image_window);
            flip(cv_ptr->image, cv_ptr->image, 1);
            imshow(thermal_image_window, cv_ptr->image);

            string binary_image_window = "binary_image";
            namedWindow(binary_image_window);
            moveWindow(binary_image_window, 565, 0);            
            imshow(binary_image_window, image_color);

            waitKey(3);
        }
        
        // Converting image to msg 
        std_msgs::Header header = msg->header; 
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_color);

        if (debug_publisher_) {
            sensor_msgs::Temperature measure_debug;
            measure_debug.header = header;
            measure_debug.temperature = max_temp_;
            measure_debug.variance = thermal_threshold_;
            pub_debug_.publish(measure_debug);
        }

    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

bool Thermal::checkFireCallback(mbzirc_comm_objs::CheckFire::Request  &req, mbzirc_comm_objs::CheckFire::Response &res)
{
    res.fire_detected = detected_;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fire_detector");
    Thermal thermal;
    ros::spin();

    return 0;
}
