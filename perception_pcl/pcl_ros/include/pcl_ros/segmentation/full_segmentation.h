
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
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <vector>
#include <pcl/features/moment_of_inertia_estimation.h>
// #include "pcl_ros/segmentation/utilities.h"



typedef pcl::PointCloud<pcl::PointXYZ> PointCloudC;
typedef pcl::PointXYZ PointC;


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
    ros::Subscriber pcl_subscriber_;
    ros::Subscriber color_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    //ros::ServiceServer minimal_service_;
    ros::Publisher  plane_publisher_;
    ros::Publisher  pcloud_publisher_;
    ros::Publisher  pose_publisher_;
    tf::TransformListener listener_;
    bool continue_;

    sensor_msgs::PointCloud2 pcloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_;
    //double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    //double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();
    void pclCallback(const sensor_msgs::PointCloud2& msg); //prototype for callback of example subscriber
    void colorCallback(const std_msgs::String& msg);
    void HSVfilter(const std_msgs::String& msg);
    bool transformPCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener);
    void tfMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat);
    void transformPCloudFinal (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,sensor_msgs::PointCloud2 &out);
    bool checkPlane(const pcl::ModelCoefficients& coef);
    //prototype for callback for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-