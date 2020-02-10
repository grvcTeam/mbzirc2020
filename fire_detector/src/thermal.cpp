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
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

#include <fire_detector/CheckFire.h>

using namespace std;
using namespace cv;

Thermal::Thermal()
{
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("teraranger_evo_thermal/raw_temp_array",1,&Thermal::thermal_data,this);
    ros::Subscriber sub_dos = nh.subscribe("teraranger_evo_thermal/rgb_image",1,&Thermal::image_operations,this);
    ros::Subscriber sub_tres = nh.subscribe("ual/pose",1,&Thermal::ual_to_fire_position,this);
    ros::Subscriber sub_cuatro = nh.subscribe("scan",1,&Thermal::laser_measures,this);

    pub = nh.advertise<sensor_msgs::Image>("thermal_camera",1);
    pub_msg = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects",1);

    ros::ServiceServer srv_checkfire = nh.advertiseService("fire_detected", &Thermal::srv_callback_checkfire, this);

    ros::NodeHandle n("~");

    n.getParam("angle_amplitude",angle_amplitude);
    angle_amplitude=angle_amplitude*CONV2PNT;
    initial=LASER_RANGE/2-angle_amplitude; // Point to laser front
    n.getParam("covariance_x",sigma[0]);
    n.getParam("covariance_y",sigma[1]);
    n.getParam("covariance_z",sigma[2]);
    n.getParam("uav_id",uav_id);
    n.getParam("debug",debug);
    n.getParam("thermal_threshold",thermal_threshold);
    n.getParam("camera_config",mode);
    detected=false;
    false_negative=0;

    ros::spin();
}

// Routine to find fire in the image - Just one fire in the image
// TODO - detect more than one possible fire
void Thermal::thermal_data(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int aux; // to loop through thermal array data
    float aux_max=0;
    for (int j=0, aux=0;j<M_TEMP;j++)
    {
        for (int k=0;k<M_TEMP;k++,aux++)
        {
            // Save current temp matrix
            temp_matrix[k][j]=msg->data[aux];
            // Search hightest temp of the array
            if (msg->data[aux]>aux_max){
                aux_max=msg->data[aux];
            }
        }
    }
    max_temp=aux_max;
}

//  Routine to obtain data pose and header to create fire messages
void Thermal::ual_to_fire_position(const geometry_msgs::PoseStamped& msg)
{
    // Position in x,y,z   
    uav_position.header=msg.header;
    uav_position.point=msg.pose.position;

	tf2::Quaternion q;
	tf2::fromMsg(msg.pose.orientation,q);
	tf2::Matrix3x3 Rot_matrix(q);
    double roll, pitch, yaw;
	Rot_matrix.getRPY(roll,pitch,yaw);
    uav_yaw = yaw;
}

//Routine to connect and obtain data from laser scanner and obtaining an average of the values
void Thermal::laser_measures(const sensor_msgs::LaserScan& msg)
{  
    float sum_measurement=0.0;
    int n_measurement=0;
    // Reading every measure established and calculating the average
    for (int i=0;i<(angle_amplitude*2);i++,n_measurement++)
    {
        if (msg.ranges[initial+i]>msg.range_min and msg.ranges[initial+i]<msg.range_max){
            sum_measurement=msg.ranges[initial+i]+sum_measurement;
        }
    }    
    laser_measurement=sum_measurement/n_measurement;
}

//  Routine to process the image and determine if there is fire and where
void Thermal::image_operations(const sensor_msgs::ImageConstPtr& msg)
{  
    // cout<<uav<<endl;
    float x_comp, y_comp;
    float cx[SCALE_FACTOR],cy[SCALE_FACTOR];
    vector<vector<Point>> outline;
    Point center,sum;
    therm = Mat::zeros(Size(M_TEMP*SCALE_FACTOR,M_TEMP*SCALE_FACTOR), CV_8UC1); // TODO Why CV_8UC1

    //Main routine 
    try{
        // Convert msg from ros topic to image 
        cv_ptr=cv_bridge::toCvCopy(msg,"8UC3");
        // Routine to obtain a black & white filter, 
        // Black=there is no fire
        // White=pixel temperature is bigger than thermal threeshold 
        for (int i=0;i<M_TEMP*SCALE_FACTOR;i++){
            for (int j=0;j<M_TEMP*SCALE_FACTOR;j++){
                if (temp_matrix[int(floor(i/SCALE_FACTOR))][int(floor(j/SCALE_FACTOR))]>=thermal_threshold || (false_negative<=MAX_FILTER_NEGATIVES && temp_matrix[int(floor(i/SCALE_FACTOR))][int(floor(j/SCALE_FACTOR))]>=max_temp-5))
                { 
                    *((uint8_t *)(therm.data + M_TEMP*SCALE_FACTOR * i + j)) =255;
                }
            }
        }
        
        center.x=M_TEMP*SCALE_FACTOR/2.0;
        center.y=M_TEMP*SCALE_FACTOR/2.0;

        findContours(therm,outline,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
        vector<Moments> mu(outline.size());
        vector<Point2f> mc(outline.size());
        cvtColor(therm,image_color,CV_GRAY2BGR);

        // Routine to detect fire in the image
        circle(image_color,center,1,CV_RGB(0,255,0),1,25,0);

        if (max_temp>=thermal_threshold || false_negative<=MAX_FILTER_NEGATIVES){
            detected=true;

            if(max_temp<=thermal_threshold)
            {
                false_negative++;
            }else
            {
                false_negative=0;
            }
            
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
                    circle(image_color,sum,R_CIRCLE,CV_RGB(0,255,0),1,16,0);
                    circle(image_color,sum,3,CV_RGB(0,255,0),1,16,0);
                    line(image_color,center,sum,CV_RGB(0,255,0),1,16,0);

                    x_comp=center.y-sum.y;
                    y_comp=(center.x-sum.x)*-1;
                }
            }

            rec_object.header.stamp = ros::Time::now();
            rec_object.header.frame_id = uav_position.header.frame_id;
            rec_object.type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE;
            rec_object.color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
            rec_object.pose.covariance={sigma[0]*sigma[0],0,0,0,0,0,
                                        0,sigma[1]*sigma[1],0,0,0,0,
                                        0,0,sigma[2]*sigma[2],0,0,0,
                                        0, 0, 0, 0, 0,  0,
                                        0, 0, 0, 0, 0,  0,
                                        0, 0, 0, 0, 0,  0};
            rec_object.image_detection.img_height=M_TEMP;
            rec_object.image_detection.img_width=M_TEMP;
            rec_object.image_detection.v=y_comp/SCALE_FACTOR;
            rec_object.image_detection.u=x_comp/SCALE_FACTOR;
            rec_object.image_detection.height=R_CIRCLE/SCALE_FACTOR;
            rec_object.image_detection.width=R_CIRCLE/SCALE_FACTOR;

            if (mode=="DOWNWARD")
            {
                //Thermal image fields
                rec_object.image_detection.depth=uav_position.point.z;
                rec_object.image_detection.camera_direction=mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_DOWNWARD;
                // Object Detection fields
                rec_object.pose.pose.position.x=uav_position.point.x;
                rec_object.pose.pose.position.y=uav_position.point.y;
                rec_object.pose.pose.position.z=0.0;
            }
            else if (mode=="FORWARD")
            {
                //Thermal image fields
                rec_object.image_detection.depth=laser_measurement;
                rec_object.image_detection.camera_direction=mbzirc_comm_objs::ThermalImage::CAMERA_DIRECTION_FORWARD;
                //Object detection fields              
                rec_object.pose.pose.position.x=uav_position.point.x+laser_measurement*cos(uav_yaw);
                rec_object.pose.pose.position.y=uav_position.point.y+laser_measurement*sin(uav_yaw);
                rec_object.pose.pose.position.z=uav_position.point.z;
            }

            if(debug)
            {
                if(false_negative)
               {                    
                    cout<<"Possible false negative: Max temp detected-> "<<max_temp<<" | Current threshold-> "<<thermal_threshold<<endl;
                }
                else
                {
                    // Fire detected
                    cout<<"Fire_detected"<<endl;
                    cout<<"Distance to center ~ x: "<<to_string(x_comp)<<"  y:"<<to_string(y_comp)<< " total:"<< to_string(sqrt(x_comp*x_comp+y_comp*y_comp))<<endl;
                }
            }

            // Publishing the Object    
            rec_list.objects.push_back(rec_object); // TODO - Check if publish full list or current point
            rec_list.stamp = ros::Time::now();
            rec_list.agent_id = uav_id;
            pub_msg.publish(rec_list);
        }
        else
        {
            detected=false;
            if(debug)
            {
                cout<<"FIRE NOT DETECTED: Max temp detected-> "<<max_temp<<" | Current threshold-> "<<thermal_threshold<<endl;
            }
        }

        flip(image_color,image_color,-1);
        rotate(image_color,image_color,ROTATE_90_COUNTERCLOCKWISE);
        
        if (debug)
        {
            // Displaying images on screen 
            string nombre="Thermal";
            namedWindow(nombre);
            flip(cv_ptr->image,cv_ptr->image,1);
            imshow(nombre,cv_ptr->image);

            string nombre_dos="B&W";
            namedWindow(nombre_dos);
            moveWindow(nombre_dos,565,0);
            
            imshow(nombre_dos, image_color);
            waitKey(3);
        }
        // Converting image to msg 
        header = msg->header; 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_color);
        pub.publish(img_bridge.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

bool Thermal::srv_callback_checkfire(fire_detector::CheckFire::Request  &req, fire_detector::CheckFire::Response &res)
{
    res.fire=detected;
    return true;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "Thermal_cam");
    Thermal thermal;

    return 0;
}
