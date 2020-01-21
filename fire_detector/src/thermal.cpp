#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
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

float temp_matrix[32][32];
int maxim;
ros::Publisher pub;
ros::Publisher pub_msg;
float x_pose, y_pose;
geometry_msgs::PoseStamped pos;

int detection_sampling=0;
int z=0;
int last_detection=0;
int detection=0;


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
//  Routine to obtain data pose to create fire messages
void ual_to_fire_position(const geometry_msgs::PoseStamped& msg)
{
    // Position in x,y,z
    x_pose=msg.pose.position.x;
    y_pose=msg.pose.position.y;
    pos.pose=msg.pose;
    // cout<<x_pose<<endl;
}

//  Routine to process the image and determine if there is fire and where
void image_operations(const sensor_msgs::ImageConstPtr& msg)
{
    
    int thermal_threshold=70;  //Thermal threshold to detect fire, 70 degrees recomended

    //Index and size of the thermal window displayed
    int a=32*20,i=0,j=0,k=0;
    int x_size=32*20;
    int y_size=32*20;
    int c=0,d=0;
    int count=0, max_count=0;
    int gray_im[32][32];
    int f=0;
    int cx[20],cy[20];

    // Variables to calculate relative position of the fire
    int image_center_x=y_size/2;
    int image_center_y=x_size/2;
    float relative_pose_x;
    float relative_pose_y;
    // Pixel - metres equivalence
    float pixel_to_m=0.0001;

    // Variables to port from msg to image and operate in opencv
    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    cv_bridge::CvImage img_bridge; 
    cv_bridge::CvImageConstPtr cv_ptr;
    mbzirc_comm_objs::ObjectDetectionList rec_list;
    mbzirc_comm_objs::ObjectDetection rec_object;
    cv::Mat image_gray;
    cv::Mat therm(cv::Size(x_size,y_size), CV_8UC1);
    uint8_t *initial_therm_ptr = therm.data;
    uint8_t *current_therm_ptr;

    //Main routine 
    try{
        // Convert msg from ros topic to image 
        cv_ptr=cv_bridge::toCvCopy(msg,"8UC3");
        cvtColor(cv_ptr->image,image_gray,CV_RGB2GRAY);
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
        Point pt1,pt2;
        cv::findContours(therm,outline,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
        std::vector<cv::Moments> mu(outline.size());
        std::vector<cv::Point2f> mc(outline.size());

        // Routine to detect fire in the image
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
                pt1.x=rect.x;
                pt1.y=rect.y;
                pt2.x=rect.x+rect.width;
                pt2.y=rect.y+rect.height;
                rectangle(therm,pt1,pt2,CV_RGB(0,255,0),1);
                // circle(therm,mc[d],5,CV_RGB(255,255,255),-1);
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
        cv::flip(therm,therm,0);
        cv::rotate(therm,therm,cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow(nombre_dos, therm);
        cv::waitKey(3);

        // COnverting image to msg 
        header = msg->header; 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, therm);
        pub.publish(img_bridge.toImageMsg());

        // Publish a msg whenever a fire is detected every 10 frames
        int frame_number=10;
        detection_sampling=detection_sampling+1;
        if (detection==1 and detection_sampling>=frame_number)
        {
            // If a fire is detected
            cout<<"Fire_detected"<<endl;
            detection=0;
            detection_sampling=0;
            last_detection=last_detection+1;
            z=0;
            // Routine to display a ObjectDetection msg for every fire detected in the image
            while (z<outline.size())
            {
                // Calculating the relative fire position toward the image center
                relative_pose_x=x_pose+((y_size-cy[z])-image_center_x)*pixel_to_m;
                relative_pose_y=y_pose+((x_size-cx[z])-image_center_y)*pixel_to_m;
                rec_object.type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE;
                rec_object.relative_position.x=relative_pose_x;
                rec_object.relative_position.y=relative_pose_y;
                rec_object.pose.pose=pos.pose;
                // Publishing the Object 
                rec_list.objects.push_back(rec_object);
                pub_msg.publish(rec_list);
                z=z+1;
            }
            last_detection=detection;
        }
        else 
        {
            last_detection=0;
            cout<<"Waiting"<<endl;
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
    ros::init(argc,argv,"Thermal_cam");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("teraranger_evo_thermal/raw_temp_array",10,thermal_data);
    ros::Subscriber sub_dos = n.subscribe("teraranger_evo_thermal/rgb_image",10,image_operations);
    ros::Subscriber sub_tres = n.subscribe("/ual/pose",10,ual_to_fire_position);
    
    pub = n.advertise<sensor_msgs::Image>("thermal_camera", 10);
    pub_msg = n.advertise<mbzirc_comm_objs::ObjectDetectionList>("fire_detected",10);
    ros::spin();
}