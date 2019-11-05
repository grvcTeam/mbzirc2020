/*!
 *      @file  bricks_detection_handler.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  16/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

#include <geometry_msgs/PoseArray.h>

#include <bricks_detection/bricks_detection.h>
#include <bricks_detection/types/image_item.h>

#include "bricks_detection_handler.h"

namespace mbzirc
{
BricksDetectionHandler::BricksDetectionHandler(std::string) : _nh("~"), _use_pointcloud(false)
{
   loadParameters();
   loadServices();

   _tfListener = new tf2_ros::TransformListener(_tf_buffer);

   loadTopics();

   _pcloud_filter_f = boost::bind(&BricksDetectionHandler::filtersReconfigureCb, this, _1, _2);
   _pcloud_filter_rconfig_server.setCallback(_pcloud_filter_f);
}

BricksDetectionHandler::~BricksDetectionHandler() {}

void BricksDetectionHandler::loadParameters()
{
   _nh.param<bool>("use_pointcloud", _use_pointcloud, false);

   _nh.param<std::string>("tf_prefix", _tf_prefix, "");
   if (_tf_prefix != "") _tf_prefix += "/";

   _nh.param<std::string>("rgb_img_topic", _image_topic, "/camera/color/image_rect_color");
   _nh.param<std::string>("rgb_info_topic", _image_info_topic, "/camera/color/camera_info");
   _nh.param<std::string>("pcloud_topic", _pcloud_topic, "/camera/depth_registered/points");

   _nh.param<std::string>(
       "colors_json", _colors_json,
       "/home/rcaballero/Projects/Mbzirc2020/team_ws/src/mbzirc2020/bricks_detection/cfg/bags_colors.json");

   ROS_INFO_STREAM("Parameters loaded!");
}

void BricksDetectionHandler::loadServices()
{
   _use_pointcloud_srv = _nh.advertiseService("use_pointcloud", &BricksDetectionHandler::usePointcloudCb, this);
}

void BricksDetectionHandler::loadTopics(const bool set_publishers)
{
   if (_use_pointcloud)
   {
      _pcloud2_sub =
          _nh.subscribe<sensor_msgs::PointCloud2>(_pcloud_topic, 1, &BricksDetectionHandler::pointcloudCb, this);
   }
   else
   {
      _rgb_sub = _nh.subscribe<sensor_msgs::Image>(_image_topic, 1, &BricksDetectionHandler::rgbImageCb, this);
      _rgb_info_sub =
          _nh.subscribe<sensor_msgs::CameraInfo>(_image_info_topic, 1, &BricksDetectionHandler::cameraInfoCb, this);
   }

   ROS_INFO_STREAM("Subscribed topics loaded!");

   if (set_publishers)
   {
      _rgb_pub = _nh.advertise<sensor_msgs::Image>("bricks/filtered_image", 1);

      _pcloud2_red_pub    = _nh.advertise<sensor_msgs::PointCloud2>("bricks/red", 1);
      _pcloud2_blue_pub   = _nh.advertise<sensor_msgs::PointCloud2>("bricks/blue", 1);
      _pcloud2_orange_pub = _nh.advertise<sensor_msgs::PointCloud2>("bricks/orange", 1);
      _pcloud2_green_pub  = _nh.advertise<sensor_msgs::PointCloud2>("bricks/green", 1);

      _detected_bricks_pub      = _nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("bricks", 1);
      _detected_bricks_pose_pub = _nh.advertise<geometry_msgs::PoseArray>("bricks/pose", 1);

      ROS_INFO_STREAM("Advertised topics loaded!");
   }
}

void BricksDetectionHandler::filtersReconfigureCb(bricks_detection::reconfig_filtersConfig& config, uint32_t)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   if (config.colors_json_path != "default") _bricks_detection->color_filtering->addHSVFilter(config.colors_json_path);

   _bricks_detection->shape_detector->setMinArea(config.min_area);
   _bricks_detection->shape_detector->setPolyEpsilon(config.poly_epsilon);

   _bricks_detection->color_filtering->setMinPointsPerColor(config.min_points_per_color);

   _bricks_detection->distance_filtering->toggleFilters(config.enable_voxel_grid, config.enable_distance_filter,
                                                        config.enable_sor);
   if (config.enable_voxel_grid)
   {
      _bricks_detection->distance_filtering->voxel_grid.setLeafSize(config.voxel_leaf_size, config.voxel_leaf_size,
                                                                    config.voxel_leaf_size);
   }

   if (config.enable_distance_filter)
   {
      _bricks_detection->distance_filtering->z_filtering.setFilterLimits(config.min_distance_limit,
                                                                         config.max_distance_limit);
   }

   if (config.enable_sor)
   {
      _bricks_detection->distance_filtering->sor_filtering.setMeanK(config.sor_mean_k);
      _bricks_detection->distance_filtering->sor_filtering.setStddevMulThresh(config.sor_std_dev_mul_thresh);
   }

   _bricks_detection->plane_detector->toggle(config.enable_plane_segmentation);
   _bricks_detection->plane_detector->setMaxIterations(config.max_ransac_iterations);
   _bricks_detection->plane_detector->setMaxCoefs(config.plane_coef0, config.plane_coef1, config.plane_coef2);
}

bool BricksDetectionHandler::usePointcloudCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
   _use_pointcloud = req.data;

   _rgb_sub.shutdown();
   _pcloud2_sub.shutdown();
   loadTopics(false);

   ROS_INFO("Setting use_pointcloud to %s", _use_pointcloud ? "true" : "false");

   res.success = true;
   return true;
}

void BricksDetectionHandler::rgbImageCb(const sensor_msgs::Image::ConstPtr& image_msg)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   try
   {
      geometry_msgs::TransformStamped camera_link_tf =
          _tf_buffer.lookupTransform(image_msg->header.frame_id, "map", ros::Time(0));

      tf2::Stamped<tf2::Transform> camera_link_tf2;
      tf2::fromMsg(camera_link_tf, camera_link_tf2);

      _camera_parameters.R = camera_link_tf2.getBasis();
      _camera_parameters.T = camera_link_tf2.getOrigin();
   }
   catch (tf2::TransformException& e)
   {
      ROS_WARN("%s", e.what());
      return;
   }

   cv_bridge::CvImagePtr image_ptr;
   try
   {
      image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_WARN("cv_bridge exception: %s", e.what());
      return;
   }

   cv::Mat filtered_img;
   std::vector<ImageItem> detected_items;
   _bricks_detection->processData(image_ptr->image, filtered_img, detected_items);

   const tf2::Matrix3x3 Rt  = _camera_parameters.R.transpose();
   tf2::Vector3 T_world     = Rt * _camera_parameters.T;
   tf2::Vector3 K_0         = _camera_parameters.K.getRow(0);
   tf2::Vector3 K_1         = _camera_parameters.K.getRow(1);
   const double estimated_z = 0.2;  // TODO check height

   geometry_msgs::PoseArray object_pose_list;
   object_pose_list.header.stamp    = image_msg->header.stamp;
   object_pose_list.header.frame_id = "map";

   mbzirc_comm_objs::ObjectDetectionList object_list;
   for (auto item : detected_items)
   {
      mbzirc_comm_objs::ObjectDetection object;
      object.header.stamp    = image_msg->header.stamp;
      object.header.frame_id = "map";
      object.type            = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK;
      object.color           = item.color_id;

      tf2::Vector3 ray;
      double aux = (item.centroid.y - K_1[2]) / K_1[1];

      ray[0] = (item.centroid.x - K_0[2] - K_0[1] * aux) / K_0[0];
      ray[1] = aux;
      ray[2] = 1.0;

      tf2::Vector3 ray_world = Rt * ray;
      if (ray_world[2] == 0)
      {
         ROS_WARN("geoLocate: ray_world[2] == 0");
         continue;
      }

      double lambda               = (estimated_z + T_world[2]) / ray_world[2];
      object.relative_position.x  = lambda * ray_world[0];
      object.relative_position.y  = lambda * ray_world[1];
      object.relative_position.z  = lambda * ray_world[2];
      object.pose.pose.position.x = object.relative_position.x - T_world[0];
      object.pose.pose.position.y = object.relative_position.y - T_world[1];
      object.pose.pose.position.z = object.relative_position.z - T_world[2];

      tf2::Vector3 orientation_camera(cos(item.orientation), sin(item.orientation), 0);
      tf2::Vector3 orientation_world = _camera_parameters.R * orientation_camera;
      double theta_world             = atan2(orientation_world[1], orientation_world[0]);
      object.relative_yaw            = theta_world;
      object.pose.pose.orientation.x = 0;
      object.pose.pose.orientation.y = 0;
      object.pose.pose.orientation.z = sin(0.5 * theta_world);
      object.pose.pose.orientation.w = cos(0.5 * theta_world);
      object.pose.covariance[0]      = 0.01;  // TODO: Covariance?
      object.pose.covariance[7]      = 0.01;
      object.pose.covariance[14]     = 0.01;

      // Suppose item is a rectangle:
      // P = 2 * (side_1 + side_2);
      // A = side_1 * side_2
      // side_i = (P ± sqrt(P*P - 16*A)) / 4
      aux                      = sqrt(item.perimeter * item.perimeter - 16.0 * item.area);
      double pixel_to_metric_x = lambda / K_0[0];
      double pixel_to_metric_y = lambda / K_1[1];
      object.scale.x           = pixel_to_metric_x * (item.perimeter + aux) / 4.0;
      object.scale.y           = pixel_to_metric_y * (item.perimeter - aux) / 4.0;
      object.scale.z           = estimated_z;

      object_list.objects.push_back(object);
      object_pose_list.poses.push_back(object.pose.pose);
   }

   _detected_bricks_pub.publish(object_list);
   _detected_bricks_pose_pub.publish(object_pose_list);

   cv_bridge::CvImage cv_bridge = cv_bridge::CvImage(image_msg->header, "bgr8", filtered_img);
   sensor_msgs::Image filtered_img_msg;
   cv_bridge.toImageMsg(filtered_img_msg);
   _rgb_pub.publish(filtered_img_msg);
}

void BricksDetectionHandler::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
   _camera_parameters.K = tf2::Matrix3x3(camera_info_msg->K[0], camera_info_msg->K[1], camera_info_msg->K[2],
                                         camera_info_msg->K[3], camera_info_msg->K[4], camera_info_msg->K[5],
                                         camera_info_msg->K[6], camera_info_msg->K[7], camera_info_msg->K[8]);

   ROS_INFO_STREAM("Camera Intrinsics set!");

   _rgb_info_sub.shutdown();
}

void BricksDetectionHandler::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& pcloud_msg)
{
   if (!_bricks_detection)
   {
      _bricks_detection = new BricksDetection();
      _bricks_detection->color_filtering->addHSVFilter(_colors_json);
   }

   geometry_msgs::TransformStamped baselink_tf;
   try
   {
      baselink_tf = _tf_buffer.lookupTransform(_tf_prefix + "base_link", pcloud_msg->header.frame_id, ros::Time(0));
   }
   catch (tf2::TransformException& e)
   {
      ROS_WARN("%s", e.what());
      return;
   }

   pcl::PointCloud<pcl::PointXYZRGB> pcloud;
   pcl::fromROSMsg(*pcloud_msg, pcloud);

   std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> color_pcloud_cluster;
   _bricks_detection->processData(pcloud, color_pcloud_cluster, baselink_tf);

   for (auto color_pcloud : color_pcloud_cluster)
   {
      pcl::PCLPointCloud2 pcloud2_out;
      pcl::toPCLPointCloud2(color_pcloud.second, pcloud2_out);

      sensor_msgs::PointCloud2 pcloud2_msg;
      pcl_conversions::fromPCL(pcloud2_out, pcloud2_msg);

      pcloud2_msg.header.stamp    = pcloud_msg->header.stamp;
      pcloud2_msg.header.frame_id = _tf_prefix + "base_link";

      if (color_pcloud.first == "red")
      {
         _pcloud2_red_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "blue")
      {
         _pcloud2_blue_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "orange")
      {
         _pcloud2_orange_pub.publish(pcloud2_msg);
         continue;
      }
      else if (color_pcloud.first == "green")
      {
         _pcloud2_green_pub.publish(pcloud2_msg);
         continue;
      }
   }
}
}  // namespace mbzirc