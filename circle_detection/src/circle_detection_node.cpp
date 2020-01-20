#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <iostream>

// Typedef
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxTimeSyncPolicy;

class CircleDetector
{
public:
    CircleDetector();
    ~CircleDetector();
private:
    void callbackDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr);
    void callbackSyncColorDepth(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr);
    bool hasCameraInfo();
    void get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int &pixel_row, int &pixel_col);

    image_transport::ImageTransport image_transport_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Depth camera info subscriber
    ros::Subscriber camera_info_sub_;
    // Depth image subscriber
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    // Color image subscriber
    message_filters::Subscriber<sensor_msgs::Image> color_image_sub_;

    bool has_camera_info;

    // Color and Depth images synchronizer
    message_filters::Synchronizer<ApproxTimeSyncPolicy> sync;

    // Aligned depth image camera_model
    image_geometry::PinholeCameraModel model_;

    bool publish_debug_marker_;

    // Circle Center Point Marker
    visualization_msgs::Marker circle_center_marker_;

    bool publish_debug_images_;

    // Circle center marker publisher
    ros::Publisher circle_center_marker_pub_;

    // Gaussian Blur Image Publisher
    image_transport::Publisher gaussian_blur_image_pub_;

    // Canny Edge Detector Image Publisher
    image_transport::Publisher canny_edge_image_pub_;

    // Detected Circle Image Publisher
    image_transport::Publisher detected_circles_image_pub_;


};

CircleDetector::CircleDetector():
nh_(),
pnh_("~"),
image_transport_(nh_),
publish_debug_images_(false),
publish_debug_marker_(false),
has_camera_info(false),
depth_image_sub_(nh_, "camera/aligned_depth_to_color/image_raw", 1),
color_image_sub_(nh_, "camera/color/image_raw", 1),
sync(ApproxTimeSyncPolicy(10), color_image_sub_, depth_image_sub_)
{
    camera_info_sub_ = nh_.subscribe("camera/aligned_depth_to_color/camera_info", 1, &CircleDetector::callbackDepthCameraInfo, this);
    //depth_image_sub_ = message_filters::Subscriber<sensor_msgs::Image>(nh, "depth_image_topic", 1);
    //color_image_sub_ = message_filters::Subscriber<sensor_msgs::Image>(nh_, "color_image_topic", 1);
    sync.registerCallback(boost::bind(&CircleDetector::callbackSyncColorDepth, this, _1, _2));

    pnh_.param("publish_debug_marker", publish_debug_marker_, false);
    if (publish_debug_marker_)
    {
        circle_center_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("circle_center_marker", 1);
        // Initialize circle_center_marker_
        circle_center_marker_.header.frame_id = "camera_color_optical_frame";
        circle_center_marker_.id = 1;
        circle_center_marker_.lifetime = ros::Duration(0);
        circle_center_marker_.ns = "circle_center";
        circle_center_marker_.type = visualization_msgs::Marker::SPHERE;
        circle_center_marker_.action = visualization_msgs::Marker::ADD;
        circle_center_marker_.pose.orientation.w = 1.0;
        circle_center_marker_.scale.x = circle_center_marker_.scale.y = circle_center_marker_.scale.z = 0.1;
        circle_center_marker_.color.a = 1.0;
        circle_center_marker_.color.r = 1.0;
        circle_center_marker_.color.b = circle_center_marker_.color.g = 0;
    }
    pnh_.param("publish_debug_images", publish_debug_images_, false);
    if (publish_debug_images_)
    {
        detected_circles_image_pub_ = image_transport_.advertise("detected_circles_image", 1); 
        gaussian_blur_image_pub_ = image_transport_.advertise("gaussian_blur_image", 1);
        canny_edge_image_pub_ = image_transport_.advertise("canny_edge_image", 1);
    }
}

CircleDetector::~CircleDetector()
{

}

bool CircleDetector::hasCameraInfo(){
    return has_camera_info;
}

void CircleDetector::callbackDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr)
{
    // Unsubscribe upon reception of first camera info message, because is supposed to be always the same
    camera_info_sub_.shutdown();
    ROS_INFO("Received camera_info");
    model_.fromCameraInfo(camera_info_ptr);
    has_camera_info = true;
}

void CircleDetector::callbackSyncColorDepth(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr)
{
    if (this->hasCameraInfo())
    {
        // Convert from ROS image type to OpenCV
        cv_bridge::CvImagePtr color_image_cv = cv_bridge::toCvCopy(color_img_ptr, sensor_msgs::image_encodings::RGB8);
        cv_bridge::CvImagePtr depth_image_cv = cv_bridge::toCvCopy(depth_img_ptr, sensor_msgs::image_encodings::TYPE_16UC1);

        cv_bridge::CvImage gray_image_cv;
        gray_image_cv.encoding = sensor_msgs::image_encodings::MONO8;
        cv::cvtColor(color_image_cv->image, gray_image_cv.image, CV_BGR2GRAY);

        // Apply Blur
        cv_bridge::CvImage gaussian_blur_image_cv;
        gaussian_blur_image_cv.encoding = sensor_msgs::image_encodings::MONO8;
        cv::GaussianBlur(gray_image_cv.image, gaussian_blur_image_cv.image, cv::Size(3,3), 2, 2);
        if (publish_debug_images_)
            gaussian_blur_image_pub_.publish(gaussian_blur_image_cv.toImageMsg());

        // Find fire circle using HoughCircles on color image
        std::vector<cv::Vec3f> circles_detected;
        double dp = 1; // Image_resolution / Accumulator_resolution
        int min_dist_between_circles = gray_image_cv.image.rows/8;
        double canny_edge_upper_threshold = 150;
        int accumulator_threshold = 50;
        int min_radius = 0;
        int max_radius = 0;

        if (publish_debug_images_)
        {
            cv_bridge::CvImage canny_edge_image;
            canny_edge_image.encoding = sensor_msgs::image_encodings::MONO8;
            cv::Canny(gaussian_blur_image_cv.image, canny_edge_image.image, canny_edge_upper_threshold, canny_edge_upper_threshold/2);
            canny_edge_image_pub_.publish(canny_edge_image.toImageMsg());
        }

        cv::HoughCircles(gaussian_blur_image_cv.image, circles_detected, CV_HOUGH_GRADIENT, dp, min_dist_between_circles, 
                         canny_edge_upper_threshold, accumulator_threshold, min_radius, max_radius);

        for (size_t i = 0; i < circles_detected.size(); i++)
        {
            cv::Point center(cvRound(circles_detected[i][0]), cvRound(circles_detected[i][1]));
            int radius = cvRound(circles_detected[i][2]);
            std::cout << "Circle " << i << " radius : " << radius << std::endl;
            // circle center
            cv::circle( color_image_cv->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
            // circle outline
            cv::circle( color_image_cv->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);

            // Extract circle center depth from depth image
            float circle_center_depth = (float)((depth_image_cv->image.at<uint16_t>(center))/1000.0);

            std::cout << "Circle center depth: " << circle_center_depth << std::endl;

            // Calc 3D position of circle center
            geometry_msgs::Point circle_center_3D;
            get3DPointCameraModel(circle_center_3D, circle_center_depth, center.y, center.x);
            circle_center_marker_.pose.position = circle_center_3D;
            circle_center_marker_pub_.publish(circle_center_marker_);
        }
        if (publish_debug_images_)
            detected_circles_image_pub_.publish(color_image_cv->toImageMsg());
    }
    else
    {
        ROS_WARN("No camera_info available");
    }
    
}

void CircleDetector::get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int &pixel_row, int &pixel_col)
{
    point.x = (pixel_col - model_.cx())/model_.fx() * depth;
    point.y = (pixel_row - model_.cy())/model_.fy() * depth;
    point.z = depth;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_detection_node");
    CircleDetector circle_detector;


    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}