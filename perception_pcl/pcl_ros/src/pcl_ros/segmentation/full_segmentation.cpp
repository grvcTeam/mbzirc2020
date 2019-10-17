
#include "pcl_ros/segmentation/full_segmentation.h"
/*
BRICK SEGMENTATION NODE:

Subscribers: Realsense PointCloud topic "/camera/depth_registered/points"
             Custom topic "/color2detect" which receives a string with the color to detect ("red", "green", "blue" and "orange")
Publishers:  PointCloud message to topic "/filtered_cloud" for DEBUG - it shows the pointcloud after filtering
             PointCloud message to topic "/segmented_plane" for DEBUG - it shows the pointcloud corresponding to the brick plane detected
             PoseStamped to the topic "/mir_manipulation/pregrasp_planner_pipeline/target_pose" which is the input topic for the pregrasp planner node
Notes: The subscriber topic "/color2detect" can be changed into a rosservice method. Also, both DEBUG topics can be supresssed in the final version. 

HOW DOES IT WORK:
    - The node is initiated and actively listens to the subscribing topics. It saves the pointcloud in a class variable and waits for a color to be published to initiate the segmentation.
    - Once it initiates it applies color segmentation based on the HSV values. This function, HSVfilter, will publish as ROS_INFO the detected color or if no color was detected. 
In the first case it continues to the execute function while in the second it simply returns to the main function with no messages published.
    - execute method applies VOX filter downsampling, followed by passfilter in coordinate z to remove undesired points, followed by a statistic outlier rejection filter.
    - Then, the PointCloud enters a planar segmentation loop which attempts to detect one of three planes: ground plane, brick plane (parallel to the ground) or other plane.
If the brick plane was not detected the loop continues where the detected plane is removed until a maximum of 3 segmentations. After 3 segmentations the either the brick plane was detected
or a ROS_INFO message was published saying "Could not estimate brick pose".
    - After the brick plane is detected the pose is estimated through the centroid of the PointCloud and its Inertial Moments. 
    - Finaly, the pose is rotated to match the expected from the pregrasp and published to the corresponding topic.
Notes: - The algorithm is not prepared for multiple bricks of the same color in the same plane
       - The bricks are assumed parallel to the ground and only small deviations will be accepted
       - It should detect if the brick is fully detected in the image, and, if not, join multiple PointClouds with a simillar functions as in concat.cpp
       - The function is not prepared to run multiple times if the color or the pose is not detected. This could be easily achieved by tunning the returns.

Final considerations:
    - The cpp and .h files were added to the pcl_package and the makelist files updated.
    - The pcl library was not updated further than 1.7 but the point_types_conversion.h file was updated with the most recent versions of the methods: PointCloudXYZRGBtoXYZHSV and PointXYZRGBtoXYZHSV

*/

BrickSegment::BrickSegment(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor

    initializeSubscribers(); 
    initializePublishers();
    //initializeServices();
    tf::TransformListener listener_;
    sensor_msgs::PointCloud2 pcloud_;
    continue_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ());

}

void BrickSegment::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pcl_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &BrickSegment::pclCallback, this);  
    color_subscriber_ = nh_.subscribe("/color2detect", 1, &BrickSegment::colorCallback, this);  
    // add more subscribers here, as needed
}

void BrickSegment::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    plane_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_plane", 1); 
    pcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1); 
   // color_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/color_segment", 1);
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mir_manipulation/pregrasp_planner_pipeline/target_pose", 1);
    // add more subscribers here, as needed
}

void BrickSegment::pclCallback(const sensor_msgs::PointCloud2& msg) 
{
    pcloud_ = msg;
}

void BrickSegment::colorCallback(const std_msgs::String& msg) 
{
    HSVfilter(msg);
}


void BrickSegment::HSVfilter(const std_msgs::String& msg) 
{
    float minH = 0.0, maxH = 0.0, minS = 0.0, maxS = 0.0, minV = 0.0, maxV = 0.0;
    if (msg.data.compare("red") == 0)
    {
      ROS_INFO("red color code");
      minH = 15; //going for invert
      maxH = 355;
      minS = 0.65;
      maxS = 1;
      minV = 0.55;
      maxV = 0.95;
    } else if(msg.data.compare("blue") == 0){
        ROS_INFO("blue color code");
        minH = 194;
        maxH = 214;
        minS = 0.75;
        maxS = 1;
        minV = 0.2;
        maxV = 0.6;
    } else if(msg.data.compare("green") == 0){
        ROS_INFO("green color code");
        minH = 113;
        maxH = 153;
        minS = 0.62;
        maxS = 1;
        minV = 0.47;
        maxV = 0.87;
    } else if(msg.data.compare("orange") == 0){
        ROS_INFO("orange color code");
        minH = 10;
        maxH = 30;
        minS = 0.67;
        maxS = 1;
        minV = 0.55;
        maxV = 0.95;
    } else {
        ROS_INFO("Unrecognized color code");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(pcloud_, *cloud);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV_pub(new pcl::PointCloud<pcl::PointXYZHSV>);
    PointCloudXYZRGBtoXYZHSV(*cloud, *HSV); // RGB -> HSV
    pcl::PassThrough<pcl::PointXYZHSV> pass;

    pass.setInputCloud (HSV);
    pass.setFilterFieldName ("h");
    pass.setFilterLimits (minH, maxH);
    if (msg.data.compare("red") == 0){
      pass.setFilterLimitsNegative (true);
    }

    pass.filter (*HSV_pub);

    pass.setFilterLimitsNegative (false);

    pass.setInputCloud (HSV_pub);
    pass.setFilterFieldName ("s");
    pass.setFilterLimits (minS, maxS);
    pass.filter (*HSV);

    pass.setInputCloud (HSV);
    pass.setFilterFieldName ("v");
    pass.setFilterLimits (minV, maxV);
    pass.filter (*HSV_pub);
    if  (HSV_pub->size() < 10000){
      ROS_INFO("Could not detect requested color");
      cout << HSV_pub->size();
      return;
    }
    temp_cloud->points.resize(HSV_pub->size());
    for (size_t i = 0; i < temp_cloud->points.size(); i++) 
    {
        temp_cloud->points[i].x = HSV_pub->points[i].x;
        temp_cloud->points[i].y = HSV_pub->points[i].y;
        temp_cloud->points[i].z = HSV_pub->points[i].z;
    }

    cloud_xyz_ = temp_cloud;
    execute();
    return;
}

bool BrickSegment::checkPlane(const pcl::ModelCoefficients& coef) 
{
    cout << coef.values[0];
    cout << coef.values[1];
    cout << coef.values[2];
    cout << coef.values[3];
    float constant;
    constant = fmod(fabs((coef.values[4])/(coef.values[3])),0.2);
    if ((fabs(coef.values[0]) < 0.2) && (fabs(coef.values[1]) < 0.2) && (fabs(coef.values[3]) < 0.1)){
        ROS_INFO("Ground Plane removed");
        return false;
    } else if (((fabs(coef.values[0]) < 0.2) && (fabs(coef.values[1]) < 0.2)) && (constant < 0.2)) {
        ROS_INFO("Brick Plane removed");
        return true;
    } else {
        ROS_INFO("Unrecognized plane");
        return false;
    }
}

void BrickSegment::execute() 
{
   // PointCloudC::Ptr cloud(new PointCloudC());
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    PointCloudC::Ptr subset_cloud(new PointCloudC);
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());

//voxel filter of pointcloud    
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud_xyz_);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.02);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);

    //Filtrar cloud na dimensao z
    pcl::PassThrough<PointC> pass;
    pass.setInputCloud (downsampled_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 4.0);
    pass.filter (*subset_cloud);

    // //Statistic outlier removal
    PointCloudC::Ptr cloud_filtered(new PointCloudC());
    pcl::StatisticalOutlierRemoval<PointC> sor;
    sor.setInputCloud (subset_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*downsampled_cloud);

//PUBLISH IS FOR DEBUG PURPOSES. NOT NEEDED IN REAL SCENARIO

    sensor_msgs::PointCloud2 msg_filter;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*downsampled_cloud, msg_filter);
    msg_filter.header.frame_id = "/camera_color_optical_frame"; 
    pcloud_publisher_.publish(msg_filter);
//work with pointcloud in base_link to know if plane is parallel to the ground
    transformPCloud("base_link", msg_filter, output, listener_);
    pcl::fromROSMsg(output, *downsampled_cloud);

//RANSAC plane extration. (not selecting any specific plane - the largest is chosen)
    pcl::ExtractIndices<PointC> extract;
    pcl::SACSegmentation<PointC> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    for (int i=0; i<3; i++){
      extract.setNegative (true);
      seg.setInputCloud (downsampled_cloud);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
          return;
      }
      bool check = checkPlane(*coefficients);
      if (check == true){
        extract.setNegative (false);
      }
      extract.setInputCloud(downsampled_cloud);
      extract.setIndices(inliers);
      extract.filter(*subset_cloud);
      *downsampled_cloud = *subset_cloud;
      if (check == true){
        break;
      }else if(i == 2) {
         ROS_INFO("Could not estimate brick pose");
         return;
      }
    }

//can continue the process
 //PUBLISH IS FOR DEBUG PURPOSES. NOT NEEDED IN REAL SCENARIO
    sensor_msgs::PointCloud2 msg_out2;
    pcl::toROSMsg(*subset_cloud, msg_out2);
    msg_out2.header.frame_id = "/base_link"; 
    plane_publisher_.publish(msg_out2);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (subset_cloud);
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
    pcl::compute3DCentroid(*subset_cloud,centroid);

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getMassCenter (mass_center);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    Eigen::Quaternionf quat_rot (rotational_matrix_OBB);
    geometry_msgs::PoseStamped point;
    geometry_msgs::Quaternion quaternion;

    float x_rot = 0.5;
    float y_rot = -0.5;
    float z_rot = 0.5;
    float w_rot = -0.5;

    quaternion.x = quat.x();
    if (quat.y() < 0){
      quaternion.y = -quat.y();
    } else {
      quaternion.y = quat.y();
    }
    quaternion.z = quat.z();
    quaternion.w = quat.w();
//rotation to get the correct pose for the grasping algorithm
    quaternion.w = quat.w()*w_rot - quat.x()*x_rot - quat.y()*y_rot - quat.z()*z_rot;
    quaternion.x = quat.w()*x_rot + quat.x()*w_rot + quat.y()*z_rot - quat.z()*y_rot;
    quaternion.y = quat.w()*y_rot - quat.x()*z_rot + quat.y()*w_rot + quat.z()*x_rot;
    quaternion.z = quat.w()*z_rot + quat.x()*y_rot - quat.y()*x_rot + quat.z()*w_rot;

    point.pose.position.x = centroid[0];
    point.pose.position.y = centroid[1];
    point.pose.position.z = centroid[2];
    point.pose.orientation = quaternion;
    
    point.header.frame_id = "/base_link"; 
    point.header.stamp = ros::Time::now();
    pose_publisher_.publish(point);
}

//functions from pcl library which were added here and changed the names. were not able to run those funtions.
bool BrickSegment::transformPCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
{
  if (in.header.frame_id == target_frame)
  {
    out = in;
    return (true);
  }

  // Get the TF transform
  tf::StampedTransform transform;
  try
  {
    tf_listener.waitForTransform (target_frame, in.header.frame_id, in.header.stamp, ros::Duration(1));
    tf_listener.lookupTransform (target_frame, in.header.frame_id, in.header.stamp, transform);
  }
  catch (const tf::TransformException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }

  // Convert the TF transform to Eigen format
  Eigen::Matrix4f eigen_transform;
  tfMatrix (transform, eigen_transform);

  transformPCloudFinal (eigen_transform, in, out);

  out.header.frame_id = target_frame;
  return (true);
}

void BrickSegment::transformPCloudFinal (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;
    
    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
  
    
    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

void BrickSegment::tfMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}


int main (int argc, char** argv)
{
 // ROS set-ups:

    ros::init(argc, argv, "BrickSegment"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    BrickSegment BrickSegment(&nh);  //instantiate an BrickSegment object and pass in pointer to nodehandle for constructor to use
    ros::spin();
    return 0;
}

