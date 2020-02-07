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
#include <thermal.h>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

using namespace std;
using namespace cv;

Thermal::Thermal()
{
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("teraranger_evo_thermal/raw_temp_array",1,&Thermal::thermal_data,this);
    ros::Subscriber sub_dos = nh.subscribe("teraranger_evo_thermal/rgb_image",1,&Thermal::image_operations,this);
    ros::Subscriber sub_tres = nh.subscribe("ual/pose",1,&Thermal::ual_to_fire_position,this);
    ros::Subscriber sub_cuatro = nh.subscribe("scan",1,&Thermal::laser_measures,this);
    nh.getParam("thermal/debug",debug);    

    pub = nh.advertise<sensor_msgs::Image>("thermal_camera",1);
    pub_msg = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects",1);
    ros::spin();
}

// Routine to find fire in the image
void Thermal::thermal_data(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int minim=MIN_TEMP_DEFAULT;
    int fila,col;
    int aux;
    maxim=0;
    // Obtaining temperature array
    for (int i=0; i<msg->data.size(); i++){     
        if (msg->data[i]>maxim){
            fila=int(i/M_TEMP);
            col=i-fila*M_TEMP;
            maxim=msg->data[i];
        }
        else if (msg->data[i]<minim){
            minim=msg->data[i];
        }
        aux=0;
        for (int b=0;b<M_TEMP;b++){
             for (int a=0;a<M_TEMP;a++){
                temp_matrix[a][b]=msg->data[aux];
                aux=aux+1;
             }
        }
    }
}

//  Routine to obtain data pose and header to create fire messages
void Thermal::ual_to_fire_position(const geometry_msgs::PoseStamped& msg)
{
    // Position in x,y,z    
    header_pose=msg.header; 
    // cout<<header_pose<<endl;
    x_pose=msg.pose.position.x;
    y_pose=msg.pose.position.y;
    z_pose=msg.pose.position.z;
    double roll_aux, pitch_aux, yaw_aux;
	tf2::Quaternion q;
	tf2::fromMsg(msg.pose.orientation,q);
	tf2::Matrix3x3 Rot_matrix(q);
	Rot_matrix.getRPY(roll_aux,pitch_aux,yaw_aux);

    yaw = yaw_aux;
    pos.pose=msg.pose;
}

//Routine to connect and obtain data from laser scanner and obtaining an average of the values
void Thermal::laser_measures(const sensor_msgs::LaserScan& msg)
{
    ros::NodeHandle t;
    int angle_amplitude;

    t.getParam("thermal/angle_amplitude",angle_amplitude);
    angle_amplitude=angle_amplitude*CONV2PNT;

    float range_min,range_max,increment_angle;
    int sum=0;
    int initial=LASER_RANGE/2; // Point to laser front
    laser_measurement=0;

    range_min=msg.range_min;
    range_max=msg.range_max;
    increment_angle=msg.angle_increment;
    initial=initial-angle_amplitude;
    // Reading every measure established and calculating the average
    for (int i=0;i<(angle_amplitude*2);i++)
    {
        if (msg.ranges[initial+i]>range_min and msg.ranges[initial+i]<range_max){
            laser_measurement=msg.ranges[initial+i]+laser_measurement;
            sum=sum+1;
        }
    }
    laser_measurement=laser_measurement/sum;
}

//  Routine to process the image and determine if there is fire and where
void Thermal::image_operations(const sensor_msgs::ImageConstPtr& msg)
{
    ros::NodeHandle t;
    
    // Get parameters from launcher
    int thermal_threshold;
    float sigma_x,sigma_y,sigma_z;
    string mode;
    bool detection=false;
    t.getParam("thermal/thermal_threshold",thermal_threshold);
    t.getParam("thermal/covariance_x",sigma_x);
    t.getParam("thermal/covariance_y",sigma_y);
    t.getParam("thermal/covariance_z",sigma_z);
    t.getParam("thermal/camera_config",mode);    
    t.getParam("thermal/uav_id",uav_id);
    // cout<<uav<<endl;
    //Index and size of the thermal window displayed
    int a=M_TEMP*SCALE_FACTOR;
    int x_size=M_TEMP*SCALE_FACTOR;
    int y_size=M_TEMP*SCALE_FACTOR;
    int count=0, max_count=0;
    int gray_im[M_TEMP][M_TEMP];
    int cx[SCALE_FACTOR],cy[SCALE_FACTOR];
    float image_center_x=x_size/2;
    float image_center_y=y_size/2;
    // Variables to port from msg to image and operate in opencv
    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    cv_bridge::CvImage img_bridge; 
    cv_bridge::CvImageConstPtr cv_ptr;
    mbzirc_comm_objs::ObjectDetectionList rec_list;
    mbzirc_comm_objs::ObjectDetection rec_object;
    Mat image_color;
    Mat therm(Size(x_size,y_size), CV_8UC1);
    uint8_t *initial_therm_ptr = therm.data;
    uint8_t *current_therm_ptr;

    //Main routine 
    try{
        // Convert msg from ros topic to image 
        cv_ptr=cv_bridge::toCvCopy(msg,"8UC3");
        // Routine to obtain a black & white filter, 
        // Black=there is no fire
        // White=pixel temperature is bigger than thermal threeshold 
        for (int i=0;i<a;i++){
            for (int j=0;j<a;j++){
                if (temp_matrix[int(floor(i/SCALE_FACTOR))][int(floor(j/SCALE_FACTOR))]>thermal_threshold)
                    {
                        
                        current_therm_ptr = (uint8_t *)(initial_therm_ptr + x_size * i + j);
                        *(current_therm_ptr) =255;
                    }
                    else
                    {
                        current_therm_ptr = (uint8_t *)(initial_therm_ptr + y_size * i + j);
                        *(current_therm_ptr) = 0;
                    }
                }
            }
        
        vector<vector<Point>> outline;
        vector<vector<Point>> rect;
        Point pt1,pt2,center,sum;
        center.x=image_center_y;
        center.y=image_center_x;

        findContours(therm,outline,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
        vector<Moments> mu(outline.size());
        vector<Point2f> mc(outline.size());
        cvtColor(therm,image_color,CV_GRAY2BGR);

        float distance;
        float x_comp;
        float y_comp;
        float radio=R_CIRCLE;
        // Routine to detect fire in the image
         circle(image_color,center,1,CV_RGB(0,255,0),1,25,0);
        if (maxim>thermal_threshold){
            detection=1;
            // Calculating moments and centers in the fire
            for (int d=0;d<outline.size();d++){
                // Obtaining centers in the fire
                mu[d]=moments(outline[d], false);
                mc[d]=Point2f(mu[d].m10/mu[d].m00 , mu[d].m01/mu[d].m00);
                cx[d]=mu[d].m10/mu[d].m00;
                cy[d]=mu[d].m01/mu[d].m00;
                // Painting a rectangle in the fire
                Rect rect=boundingRect(outline[d]);
                circle(image_color,mc[d],2,CV_RGB(0,255,0),1,16,0);
            }
            for (int d=0;d<outline.size();d++)
            {
                sum.x=cx[d]+sum.x;
                sum.y=cy[d]+sum.y;
                if (d==outline.size()-1){
                    sum.y=sum.y/outline.size();
                    sum.x=sum.x/outline.size();
                    circle(image_color,sum,radio,CV_RGB(0,255,0),1,16,0);
                    circle(image_color,sum,3,CV_RGB(0,255,0),1,16,0);
                    line(image_color,center,sum,CV_RGB(0,255,0),1,16,0);

                    x_comp=center.y-sum.y;
                    y_comp=(center.x-sum.x)*-1;
                    distance=sqrt(x_comp*x_comp+y_comp*y_comp);
                }
            }
        }

        if (debug)
        {
            // Displaying images on screen 
            string nombre="Thermal";
            namedWindow(nombre);
            rotate(cv_ptr->image,cv_ptr->image,ROTATE_180);
            imshow(nombre,cv_ptr->image);

            string nombre_dos="B&W";
            namedWindow(nombre_dos);
            moveWindow(nombre_dos,565,0);
            flip(image_color,image_color,0);
            rotate(image_color,image_color,ROTATE_90_COUNTERCLOCKWISE);
            imshow(nombre_dos, image_color);
            waitKey(3);
        }

        // COnverting image to msg 
        header = msg->header; 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, image_color);
        pub.publish(img_bridge.toImageMsg());
        // Publish when a fire is detected
        if (detection)
        {
            if(debug)
            {
                // If a fire is detected
                cout<<"Fire_detected"<<endl;
                string x_dis = to_string(x_comp);
                string y_dis = to_string(y_comp);
                // putText(image_color,s,center,FONT_HERSHEY_SIMPLEX,1,CV_RGB(100,100,0),1,1,0);
                cout<<"Distance to center ~ x: "<<x_dis<<"  y:"<<y_dis<<endl;
            }

            rec_object.header.stamp = ros::Time::now();
            rec_object.header.frame_id = header_pose.frame_id;
            rec_object.type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE;
            rec_object.color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
            rec_object.pose.covariance={sigma_x*sigma_x,0,0,0,0,0,
                                        0,sigma_y*sigma_y,0,0,0,0,
                                        0,0,sigma_z*sigma_z,0,0,0,
                                        0, 0, 0, 0, 0,  0,
                                        0, 0, 0, 0, 0,  0,
                                        0, 0, 0, 0, 0,  0};
            rec_object.image_detection.img_height=y_size/SCALE_FACTOR;
            rec_object.image_detection.img_width=x_size/SCALE_FACTOR;
            rec_object.image_detection.v=y_comp/SCALE_FACTOR;
            rec_object.image_detection.u=x_comp/SCALE_FACTOR;
            rec_object.image_detection.height=radio/SCALE_FACTOR;
            rec_object.image_detection.width=radio/SCALE_FACTOR;

            if (mode=="DOWNWARD")
            {
            //Thermal image fields
            rec_object.image_detection.depth=z_pose;
            rec_object.image_detection.camera_direction=mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_DOWNWARD;
            // Object Detection fields
            rec_object.pose.pose.position.x=x_pose;
            rec_object.pose.pose.position.y=y_pose;
            rec_object.pose.pose.position.z=0.0;
            }
            else if (mode=="FORWARD")
            {
            //Thermal image fields
            rec_object.image_detection.depth=laser_measurement;
            rec_object.image_detection.camera_direction=mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_FORWARD;
           //Object detection fields

            
            rec_object.pose.pose.position.x=x_pose+laser_measurement*cos(yaw);
            rec_object.pose.pose.position.y=y_pose+laser_measurement*sin(yaw);
            rec_object.pose.pose.position.z=z_pose;
            }
            // Publishing the Object    
            rec_list.objects.push_back(rec_object);
            rec_list.stamp = ros::Time::now();
            rec_list.agent_id = uav_id;
            pub_msg.publish(rec_list);
        }
        else 
        {
            if(debug)
            {
                cout<<"."<<endl;
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Thermal_cam");
    Thermal thermal;
    ros::spin();
    return 0;
}
