#include "pcl_ros/segmentation/concat_test.h"

ConCat::ConCat(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor

    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    //initialize variables here, as needed
    //val_to_remember_=0.0; 
    //sensor_msgs::PointCloud2 pcloud_;
    sensor_msgs::PointCloud2 pcloud_;
    sensor_msgs::PointCloud2 zero_;
    tf::TransformListener listener_;
    counter_ = 0.0;
    

}

void ConCat::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pcl2_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &ConCat::pcl2Callback, this);  
    //pcl2_subscriber_ = nh_.subscribe("/cropbox/output", 1, &BrickSegment::pcl2Callback, this);  
    // add more subscribers here, as needed
}

void ConCat::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    plane_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_plane", 1); //fisrt segmentation
    plane_publisher2_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_plane2", 1); //second segmentation
    pcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1); 
    // add more subscribers here, as needed
}

bool ConCat::transformPCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
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

void ConCat::transformPCloudFinal (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,sensor_msgs::PointCloud2 &out)
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

void ConCat::tfMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
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

sensor_msgs::PointCloud2 ConCat::FilterCloud(const sensor_msgs::PointCloud2& msg)
{
    PointCloudC::Ptr cloud(new PointCloudC);
    PointCloudC::Ptr downsampled_cloud(new PointCloudC);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::fromROSMsg(msg, *cloud);
//Filtrar cloud na dimensao z
    pcl::PassThrough<PointC> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*downsampled_cloud);
//voxel filter of pointcloud
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(downsampled_cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.02);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*cloud);
    pcl::toROSMsg(*cloud,ros_cloud);
    return ros_cloud;

}

void ConCat::pcl2Callback(const sensor_msgs::PointCloud2& msg) 
{  
    sensor_msgs::PointCloud2 pcl_out1;
    sensor_msgs::PointCloud2 filtered_msg;
    filtered_msg = FilterCloud(msg);

    if (counter_ == 0){
      counter_ = counter_ +1;

      transformPCloud("/base_link", filtered_msg, pcl_out1, listener_);
      //pcl_out1.header.frame_id = "/base_link";
      zero_.header.frame_id = msg.header.frame_id;
      pcloud_ = pcl_out1;
      sleep(2);
      return;
    }

    sensor_msgs::PointCloud2 pcl_out2;
    PointCloudC::Ptr cloud_a(new PointCloudC);
    PointCloudC::Ptr cloud_b(new PointCloudC);
    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(pcloud_, *cloud_a);
    transformPCloud("base_link", filtered_msg, pcl_out2, listener_);
    pcl::fromROSMsg(pcl_out2, *cloud_b);

    *cloud_a += *cloud_b;
    sleep(2);
    pcl::toROSMsg(*cloud_a,output);
    //output.header.frame_id = "base_link";
    pcloud_ = output;
    counter_ = counter_ +1;
    if (counter_ > 3){
        execute();
        counter_ = 0;
        pcloud_ = zero_;
    }

}

void ConCat::execute() 
{
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(pcloud_, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices());
    PointCloudC::Ptr subset_cloud(new PointCloudC);
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    PointCloudC::Ptr final_paint(new PointCloudC());
   // PointCloudC::Ptr cloud(new PointCloudC());

//     //Filtrar cloud na dimensao z
//     pcl::PassThrough<PointC> pass;
//     pass.setInputCloud (cloud);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (0.0, 4.0);
//     //pass.setFilterLimitsNegative (true);
//     pass.filter (*downsampled_cloud);
    
// //voxel filter of pointcloud
//     pcl::VoxelGrid<PointC> vox;
//     vox.setInputCloud(downsampled_cloud);
//     double voxel_size;
//     ros::param::param("voxel_size", voxel_size, 0.02);
//     vox.setLeafSize(voxel_size, voxel_size, voxel_size);
//     vox.filter(*cloud);


    // //Statistic outlier removal
    PointCloudC::Ptr cloud_filtered(new PointCloudC());
    pcl::StatisticalOutlierRemoval<PointC> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*downsampled_cloud);

    sensor_msgs::PointCloud2 msg_filter;
    pcl::toROSMsg(*downsampled_cloud, msg_filter);
    pcloud_publisher_.publish(msg_filter);


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
    plane_publisher2_.publish(msg_out2); //second segmentation

    for (size_t ii = 0; ii < inliers->indices.size (); ++ii)
    {
        downsampled_cloud->points[inliers->indices[ii]].r = 0;
        downsampled_cloud->points[inliers->indices[ii]].g = 0;
        downsampled_cloud->points[inliers->indices[ii]].b = 255;
    }   
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(*final_paint); //fisrt segmentation
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

    ros::init(argc, argv, "ConCat"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ConCat ConCat(&nh);  //instantiate an BrickSegment object and pass in pointer to nodehandle for constructor to use
    ros::Rate r(0.1);
    // while (ros::ok())
    // {
    //     // Publish the message.
    //     //BrickSegment.execute();

    //     ros::spinOnce();
    //     r.sleep();
    // }
    ros::spin();
    return 0;
}

