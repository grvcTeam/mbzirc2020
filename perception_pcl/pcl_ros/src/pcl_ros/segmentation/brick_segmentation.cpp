
#include "pcl_ros/segmentation/brick_segmentation.h"

BrickSegment::BrickSegment(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor

    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    //initialize variables here, as needed
    //val_to_remember_=0.0; 
    sensor_msgs::PointCloud2 pcloud_;

}

void BrickSegment::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pcl2_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &BrickSegment::pcl2Callback, this);  
    //pcl2_subscriber_ = nh_.subscribe("/cropbox/output", 1, &BrickSegment::pcl2Callback, this);  
    // add more subscribers here, as needed
}

void BrickSegment::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    plane_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_plane", 1); 
    plane_publisher2_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_plane2", 1); 
    point_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/centroid", 1);
    // add more subscribers here, as needed
}

void BrickSegment::pcl2Callback(const sensor_msgs::PointCloud2& msg) 
{
    pcloud_ = msg;
}

void BrickSegment::execute() 
{
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(pcloud_, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices());
    PointCloudC::Ptr subset_cloud(new PointCloudC);

// //Statistic outlier removal
//     PointCloudC::Ptr cloud_filtered(new PointCloudC());
//     pcl::StatisticalOutlierRemoval<PointC> sor;
//     sor.setInputCloud (cloud);
//     sor.setMeanK (50);
//     sor.setStddevMulThresh (1.0);
//     sor.filter (*cloud_filtered);
//voxel filter of pointcloud
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    PointCloudC::Ptr final_paint(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.02);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
    //downsampled_cloud = cloud;


    pcl::ExtractIndices<PointC> extract;
    // Create the segmentation object
    pcl::SACSegmentation<PointC> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.07);

    seg.setInputCloud (downsampled_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("FIRST SEGMENT Could not estimate a planar model for the given dataset.");
        //return (-1);
    }
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter(*subset_cloud);

    //TEST BY RESEGMENTING
    PointCloudC::Ptr final_cloud(new PointCloudC);
    seg.setDistanceThreshold (0.015);
    seg.setInputCloud (subset_cloud);
    seg.segment (*inliers2, *coefficients2);

    if (inliers2->indices.size () == 0)
    {
        PCL_ERROR ("SECOND SEGMENT Could not estimate a planar model for the given dataset.");
        //return (-1);
    }    
    //PLANE PAINTING
    for (size_t i = 0; i < inliers2->indices.size (); ++i)
    {
        subset_cloud->points[inliers2->indices[i]].r = 255;
        subset_cloud->points[inliers2->indices[i]].g = 0;
        subset_cloud->points[inliers2->indices[i]].b = 0;
    }

    extract.setInputCloud(subset_cloud);
    extract.setIndices(inliers2);
    extract.setNegative (false);
    extract.filter(*final_cloud);


//PLANE PAINTING
 
    sensor_msgs::PointCloud2 msg_out2;
    pcl::toROSMsg(*final_cloud, msg_out2);
    plane_publisher2_.publish(msg_out2);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*final_cloud,centroid); 
    geometry_msgs::PointStamped point;
    point.point.x = centroid[2];
    point.point.y = -centroid[0];
    point.point.z = -centroid[1];
    point.header.frame_id = "/camera_aligned_depth_to_infra1_frame"; 
    point.header.stamp = ros::Time::now();
 //   point_publisher_.publish(point);


    for (size_t ii = 0; ii < inliers->indices.size (); ++ii)
    {
        downsampled_cloud->points[inliers->indices[ii]].r = 0;
        downsampled_cloud->points[inliers->indices[ii]].g = 0;
        downsampled_cloud->points[inliers->indices[ii]].b = 255;
    }   
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(*final_paint);
//Statistic outlier removal
    // PointCloudC::Ptr cloud_filtered2(new PointCloudC());
    // pcl::StatisticalOutlierRemoval<PointC> sor2;
    // sor2.setInputCloud (final_paint);
    // sor2.setMeanK (30);
    // sor2.setStddevMulThresh (.1);
    // sor2.filter (*cloud_filtered2);
    // sensor_msgs::PointCloud2 msg_out;
    // pcl::toROSMsg(*cloud_filtered2, msg_out);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*final_paint, msg_out);

    plane_publisher_.publish(msg_out);
    //return (0);
}


int main (int argc, char** argv)
{
 // ROS set-ups:

    ros::init(argc, argv, "BrickSegment"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    BrickSegment BrickSegment(&nh);  //instantiate an BrickSegment object and pass in pointer to nodehandle for constructor to use
    ros::Rate r(2);
    while (ros::ok())
    {
        // Publish the message.
        BrickSegment.execute();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

