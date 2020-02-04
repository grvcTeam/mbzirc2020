#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/MultiArrayDimension.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdint.h>
#include <vector>
using namespace std;
using namespace cv;
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/DetectTypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



int detection_sampling=0;
int z=0;
int last_detection=0;
int detection=0;
int maxim;
float temp_matrix[32][32];
float x_pose, y_pose, z_pose;
float yaw;
float laser_measurement;
string uav_id;

geometry_msgs::PoseStamped pos;
std_msgs::Header header_pose;
ros::Publisher pub;
ros::Publisher pub_msg;


// Routine to find fire in the image
void thermal_data(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int minim=1000;
    int fila,col;
    int i=0,a=0,b=0,j=0;
    maxim=0;
    // Obtaining temperature array
    for (i=0; i<msg->data.size(); i++){
        if (msg->data[i]>maxim){
            fila=int(i/32);
            col=i-fila*32;
            maxim=msg->data[i];
        }
        else if (msg->data[i]<minim){
            minim=msg->data[i];
        }
        j=0;
        for (b=0;b<32;b++){
             for (a=0;a<32;a++){
                temp_matrix[a][b]=msg->data[j];
                j=j+1;
             }
        }
    }
}


//  Routine to obtain data pose and header to create fire messages
void ual_to_fire_position(const geometry_msgs::PoseStamped& msg)
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
void laser_measures(const sensor_msgs::LaserScan& msg)
{
    ros::NodeHandle t;
    int angle_amplitude;
    t.getParam("/thermal/angle_amplitude",angle_amplitude);
    float range_min,range_max,increment_angle;
    int cuentas=0,i=0;
    int initial=360;
    laser_measurement=0;
    range_min=msg.range_min;
    range_max=msg.range_max;
    increment_angle=msg.angle_increment;

    initial=initial-angle_amplitude;
    // Reading every measure established and calculating the average
    for (i=0;i<(angle_amplitude*2);i++)
    {
        if (msg.ranges[initial+i]>range_min and msg.ranges[initial+i]<range_max){
            laser_measurement=msg.ranges[initial+i]+laser_measurement;
            cuentas=cuentas+1;
        }
    }
    laser_measurement=laser_measurement/cuentas;
    }


//  Routine to process the image and determine if there is fire and where
void image_operations(const sensor_msgs::ImageConstPtr& msg)
{
    ros::NodeHandle t;
    // Get parameters from launcher
    int thermal_threshold;
    int frame_number;
    float sigma_x,sigma_y,sigma_z;
    string mode;
    t.getParam("/thermal/thermal_threshold",thermal_threshold);
    t.getParam("/thermal/sampling",frame_number);    
    t.getParam("/thermal/covariance_x",sigma_x);
    t.getParam("/thermal/covariance_y",sigma_y);
    t.getParam("/thermal/covariance_z",sigma_z);
    t.getParam("/thermal/camera_config",mode);    
    //Index and size of the thermal window displayed
    int a=32*20,i=0,j=0,k=0;
    int x_size=32*20;
    int y_size=32*20;
    int c=0,d=0;
    int count=0, max_count=0;
    int gray_im[32][32];
    int f=0;
    int cx[20],cy[20];
    float im_center_x=x_size/2;
    float im_center_y=y_size/2;
    // Variables to calculate relative position of the fire
    int image_center_x=y_size/2;
    int image_center_y=x_size/2;
    float relative_pose_x;
    float relative_pose_y;
    // Variables to port from msg to image and operate in opencv
    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    cv_bridge::CvImage img_bridge; 
    cv_bridge::CvImageConstPtr cv_ptr;
    mbzirc_comm_objs::ObjectDetectionList rec_list;
    mbzirc_comm_objs::ObjectDetection rec_object;
    cv::Mat image_color;
    cv::Mat therm(cv::Size(x_size,y_size), CV_8UC1);
    uint8_t *initial_therm_ptr = therm.data;
    uint8_t *current_therm_ptr;

    //Main routine 
    try{
        // Convert msg from ros topic to image 
        cv_ptr=cv_bridge::toCvCopy(msg,"8UC3");
        // Routine to obtain a black & white filter, 
        // Black=there is no fire
        // White=pixel temperature is bigger than thermal threeshold 
        for (i=0;i<a;i++){
            for (j=0;j<a;j++){
                if (temp_matrix[int(floor(i/20))][int(floor(j/20))]>thermal_threshold)
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
        
        std::vector<std::vector<cv::Point> > outline;
        std::vector<std::vector<cv::Point> > rect;
        Point pt1,pt2,center,sum;
        center.x=image_center_y;
        center.y=image_center_x;
        cv::findContours(therm,outline,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
        std::vector<cv::Moments> mu(outline.size());
        std::vector<cv::Point2f> mc(outline.size());
        cv::cvtColor(therm,image_color,CV_GRAY2BGR);

        float distance;
        float x_comp;
        float y_comp;
        int radio=70;
        // Routine to detect fire in the image
         circle(image_color,center,1,CV_RGB(0,255,0),1,25,0);
        if (maxim>thermal_threshold){
            detection=1;
            // Calculating moments and centers in the fire
            for (d=0;d<outline.size();d++){
                // Obtaining centers in the fire
                mu[d]=moments(outline[d], false);
                mc[d]=Point2f(mu[d].m10/mu[d].m00 , mu[d].m01/mu[d].m00);
                cx[d]=mu[d].m10/mu[d].m00;
                cy[d]=mu[d].m01/mu[d].m00;
                // Painting a rectangle in the fire
                Rect rect=boundingRect(outline[d]);
                circle(image_color,mc[d],2,CV_RGB(0,255,0),1,16,0);
            }
            for (d=0;d<outline.size();d++)
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
        // Displaying images on screen 
        string nombre="Thermal";
        cv::namedWindow(nombre);
        cv::rotate(cv_ptr->image,cv_ptr->image,cv::ROTATE_180);
        cv::imshow(nombre,cv_ptr->image);

        string nombre_dos="B&W";
        cv::namedWindow(nombre_dos);
        cv::moveWindow(nombre_dos,565,0);
        cv::flip(image_color,image_color,0);
        cv::rotate(image_color,image_color,cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow(nombre_dos, image_color);
        cv::waitKey(3);

        // COnverting image to msg 
        header = msg->header; 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, image_color);
        pub.publish(img_bridge.toImageMsg());
        // Publish a msg whenever a fire is detected every X frames
        detection_sampling=detection_sampling+1;
        if (detection==1 and detection_sampling>=frame_number)
        {
            // If a fire is detected
            cout<<"Fire_detected"<<endl;
            std::string x_dis = std::to_string(x_comp);
            std::string y_dis = std::to_string(y_comp);
            // putText(image_color,s,center,FONT_HERSHEY_SIMPLEX,1,CV_RGB(100,100,0),1,1,0);
            cout<<"Distance to center ~ x: "<<x_dis<<"  y:"<<y_dis<<endl;
            detection_sampling=0;
            last_detection=last_detection+1;
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
            rec_object.image_detection.img_height=y_size/20.0;
            rec_object.image_detection.img_width=x_size/20.0;
            rec_object.image_detection.v=y_comp/20.0;
            rec_object.image_detection.u=x_comp/20.0;
            rec_object.image_detection.height=radio/20.0;
            rec_object.image_detection.width=radio/20.0;

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
            detection=0;
            last_detection=detection;
        }
        else 
        {
            last_detection=0;
            cout<<"."<<endl;
        }
        last_detection=detection;
        

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


}

// Routine to read data from teraranger topics and ual pose
int main(int argc, char **argv)
{
    float thermal_thres;
    string Position_topic,Laser_topic,Temp_topic,RGB_image_topic,Advertise_topic,Detection_list_topic;
    ros::init(argc,argv,"Thermal_cam");
    ros::NodeHandle n;
    n.getParam("/thermal/Position_topic",Position_topic);
    n.getParam("/thermal/Laser_topic",Laser_topic);
    n.getParam("/thermal/Temp_topic",Temp_topic);
    n.getParam("/thermal/RGB_image_topic",RGB_image_topic);
    n.getParam("/thermal/Advertise_topic",Advertise_topic);
    n.getParam("/thermal/Detection_list_topic",Detection_list_topic);
    n.getParam("/thermal/uav_id",uav_id);

    ros::Subscriber sub = n.subscribe(Temp_topic,10,thermal_data);
    ros::Subscriber sub_dos = n.subscribe(RGB_image_topic,10,image_operations);
    ros::Subscriber sub_tres = n.subscribe(Position_topic,10,ual_to_fire_position);
    ros::Subscriber sub_cuatro = n.subscribe(Laser_topic,10,laser_measures);
   
    pub = n.advertise<sensor_msgs::Image>(Advertise_topic, 10);
    pub_msg = n.advertise<mbzirc_comm_objs::ObjectDetectionList>(Detection_list_topic,10);
    ros::spin();
}