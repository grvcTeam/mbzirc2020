
#ifndef PCL_ROS_BRICK_SEGMENTATION_H_
#define PCL_ROS_BRICK_SEGMENTATION_H_

#include "pcl_ros/pcl_nodelet.h"
#include <message_filters/pass_through.h>
#include "sensor_msgs/PointCloud2.h"

#include <pluginlib/class_list_macros.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/filters/extract_indices.h"
#include <ros/ros.h> 
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PointStamped.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
typedef pcl::PointXYZRGB PointC;

class BrickSegment
{
public:
    BrickSegment(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    void execute();
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber pcl2_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    ros::Publisher  plane_publisher_;
    ros::Publisher  plane_publisher2_;
    ros::Publisher  point_publisher_;

    sensor_msgs::PointCloud2 pcloud_;
    //double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    //double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();
    void pcl2Callback(const sensor_msgs::PointCloud2& msg); //prototype for callback of example subscriber
    
    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-