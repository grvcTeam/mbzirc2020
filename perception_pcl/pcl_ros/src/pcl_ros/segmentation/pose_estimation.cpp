
#include "pcl_ros/segmentation/pose_estimation.h"

PoseEstimator::PoseEstimator(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor

    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    //initialize variables here, as needed
    //val_to_remember_=0.0; 
    sensor_msgs::PointCloud2 pcloud_;

}

void PoseEstimator::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pcl2_subscriber_ = nh_.subscribe("/segmented_plane2", 1, &PoseEstimator::pcl2Callback, this);  
 
    // add more subscribers here, as needed
}

void PoseEstimator::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/brick_pose", 1);
    // add more subscribers here, as needed
}

// void PoseEstimator::initializeServices()
// {
//     ROS_INFO("Initializing Services");
//     minimal_service_ = nh_.advertiseService("pose_estimation_service",
//                                                    &PoseEstimator::serviceCallback,
//                                                    this);  
//     // add more services here, as needed
// }
void PoseEstimator::pcl2Callback(const sensor_msgs::PointCloud2& msg) 
{
    pcloud_ = msg;

}
// bool PoseEstimator::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
//     ROS_INFO("service callback activated");
//     execute();
//     return true;
//}
void PoseEstimator::execute() 
{
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(pcloud_, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());

    cloud_xyz->points.resize(cloud->size());
    for (size_t i = 0; i < cloud_xyz->points.size(); i++) 
    {
        cloud_xyz->points[i].x = cloud->points[i].x;
        cloud_xyz->points[i].y = cloud->points[i].y;
        cloud_xyz->points[i].z = cloud->points[i].z;
    }

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_xyz);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    Eigen::Matrix3f rotational_matrix_OBB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getMassCenter (mass_center);
    Eigen::Quaternionf quat (rotational_matrix_OBB);

    // pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    // pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    // pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    // pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    // viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    // viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    // viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    geometry_msgs::PoseStamped point;
    geometry_msgs::Quaternion quaternion;
    quaternion.x = quat.x();
    quaternion.y = quat.y();
    quaternion.z = quat.z();
    quaternion.w = quat.w();
    point.pose.position.x = centroid[0];
    point.pose.position.y = centroid[1];
    point.pose.position.z = centroid[2];
    point.pose.orientation = quaternion;
    
    point.header.frame_id = pcloud_.header.frame_id; 
    point.header.stamp = ros::Time::now();
    //if (quat.w() > 0.3)
    //{
// auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
// std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

    pose_publisher_.publish(point);
    //}


}


int main (int argc, char** argv)
{
 // ROS set-ups:
    ros::init(argc, argv, "PoseEstimator"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    PoseEstimator PoseEstimator(&nh);  //instantiate an BrickSegment object and pass in pointer to nodehandle for constructor to use
    ros::Rate r(2);
    while (ros::ok())
    {
        // Publish the message.
        PoseEstimator.execute();

        ros::spinOnce();
        r.sleep();
    }
    // ros::spin();
    return 0;
}

