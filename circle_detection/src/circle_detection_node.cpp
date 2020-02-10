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
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <iostream>
#include <string>

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
    void findCirclesColor(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr);
    void findCirclesDepth(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr);
    bool hasCameraInfo();
    void get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int &pixel_row, int &pixel_col);
    float getRadiusInMetersFromPixels(float &depth, int &radius_pixel);

    image_transport::ImageTransport image_transport_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string tf_prefix_;

    // Hough Circles parameters
    double dp_; // Inverse ratio of accumulator resolution to image resolution. The greater the value, the lower the accu resolution
    int min_dist_between_circle_center_; // Min distance between two circles centers.
    double canny_edge_upper_threshold_; // Intensity gradient threshold to detect strong edges
    int accu_th_; // Accu min value to detect a circle
    int min_radius_pixels_; // Circle min radius
    int max_radius_pixels_; // Circle max radius

    float gaussian_blur_sigma_;
    int gaussian_blur_kernel_size_;

    // Detected circles filtering
    float min_radius_meters_;
    float max_radius_meters_;
    float max_depth_th_;

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

    // Object detection list objects
    mbzirc_comm_objs::ObjectDetection object_detection_;
    mbzirc_comm_objs::ObjectDetectionList object_detection_list_;

    // Object detection list publisher
    ros::Publisher object_detection_pub_;

    // Flag to select if the color image is used to detect circles instead of the depth image
    bool use_color_image_;

    bool use_gaussian_blur_;
};

CircleDetector::CircleDetector():
nh_(),
pnh_("~"),
image_transport_(nh_),
publish_debug_images_(false),
publish_debug_marker_(false),
use_color_image_(false),
has_camera_info(false),
depth_image_sub_(nh_, "camera/aligned_depth_to_color/image_raw", 1),
color_image_sub_(nh_, "camera/color/image_rect_color", 1),
sync(ApproxTimeSyncPolicy(10), color_image_sub_, depth_image_sub_)
{
    // Read parameters
    pnh_.param<std::string>("tf_prefix", tf_prefix_, "mbzirc2020_1");
    pnh_.param<bool>("publish_debug_marker", publish_debug_marker_, false);
    pnh_.param<bool>("publish_debug_images", publish_debug_images_, false);
    pnh_.param<double>("dp", dp_, 1);
    pnh_.param<int>("min_dist_between_circle_center", min_dist_between_circle_center_, 480/8);
    pnh_.param<double>("canny_edge_upper_threshold", canny_edge_upper_threshold_, 150);
    pnh_.param<int>("accumulator_threshold", accu_th_, 50);
    pnh_.param<int>("min_radius", min_radius_pixels_, 0);
    pnh_.param<int>("max_radius", max_radius_pixels_, 0);
    pnh_.param<float>("min_radius_meters", min_radius_meters_, 0.05);
    pnh_.param<float>("max_radius_meters", max_radius_meters_, 0.6);
    pnh_.param<float>("max_depth_th", max_depth_th_, 10.0);
    pnh_.param<bool>("use_color_image", use_color_image_, true);
    pnh_.param<bool>("use_gaussian_blur", use_gaussian_blur_, false);
    pnh_.param<float>("gaussian_blur_sigma", gaussian_blur_sigma_, 2);
    pnh_.param<int>("gaussian_blur_kernel_size", gaussian_blur_kernel_size_, 3);

    std::string camera_info_topic = "camera/aligned_depth_to_color/camera_info";
    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &CircleDetector::callbackDepthCameraInfo, this);
    //depth_image_sub_ = message_filters::Subscriber<sensor_msgs::Image>(nh, "depth_image_topic", 1);
    //color_image_sub_ = message_filters::Subscriber<sensor_msgs::Image>(nh_, "color_image_topic", 1);
    sync.registerCallback(boost::bind(&CircleDetector::callbackSyncColorDepth, this, _1, _2));
    
    if (publish_debug_marker_)
    {
        circle_center_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("circle_detection/circle_center_marker", 1);
        // Initialize circle_center_marker_
        circle_center_marker_.header.frame_id = tf_prefix_ + "/camera_color_optical_frame";
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

    if (publish_debug_images_)
    {
        detected_circles_image_pub_ = image_transport_.advertise("circle_detection/detected_circles_image", 1);
        if (use_gaussian_blur_)
            gaussian_blur_image_pub_ = image_transport_.advertise("circle_detection/gaussian_blur_image", 1);
        canny_edge_image_pub_ = image_transport_.advertise("circle_detection/canny_edge_image", 1);
    }

    object_detection_.header.frame_id = tf_prefix_ + "/camera_color_optical_frame";
    object_detection_.type = object_detection_.TYPE_HOLE;
    object_detection_.color = object_detection_.COLOR_UNKNOWN;
    object_detection_pub_ = nh_.advertise<mbzirc_comm_objs::ObjectDetectionList>("sensed_objects", 1);
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
        if ( color_img_ptr->header.frame_id != (tf_prefix_ + "/camera_color_optical_frame") ) 
        {
            ROS_WARN("Received camera frame doesn't match expected one");
            return;
        }
        if (use_color_image_ == true)
            findCirclesColor(color_img_ptr, depth_img_ptr);
        else
            findCirclesDepth(color_img_ptr, depth_img_ptr);
    }
    else
    {
        ROS_WARN("No camera_info available");
    }
 
}

void CircleDetector::findCirclesDepth(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr)
{
    // Convert from ROS image type to OpenCV
    cv_bridge::CvImagePtr color_image_cv = cv_bridge::toCvCopy(color_img_ptr, sensor_msgs::image_encodings::RGB8);

    // Convert depth image to grayscale
    uint16_t min_depth_value = std::numeric_limits<uint16_t>::max();
    uint16_t max_depth_value = 0;
    float min_depth_value_meters = std::numeric_limits<float>::infinity();
    float max_depth_value_meters = 0.0;
    const uint32_t &image_width = depth_img_ptr->width;
    const uint32_t &image_height = depth_img_ptr->height;
    
    if (depth_img_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        cv_bridge::CvImagePtr depth_image_milimeters_cv = cv_bridge::toCvCopy(depth_img_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_bridge::CvImage depth_image_meters_cv;
        depth_image_meters_cv.image = cv::Mat(cv::Size(image_width, image_height), CV_32FC1);
        depth_image_meters_cv.header = depth_image_milimeters_cv->header;
        depth_image_meters_cv.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        cv_bridge::CvImage depth_gray_cv;
        depth_gray_cv.image = cv::Mat(cv::Size(image_width, image_height), CV_8UC1);
        depth_gray_cv.header = depth_image_milimeters_cv->header;
        depth_gray_cv.encoding = sensor_msgs::image_encodings::MONO8;
        uint16_t *  value_original_mm_ptr;
        float * value_converted_meters_ptr;
        uint8_t * value_converted_gray_ptr;
        int i1 = 0;
        int j1 = 0;
        // Convert depth image from milimeters to meters and also find min and max values
        for(i1 = 0; i1 < depth_image_milimeters_cv->image.rows; i1++)
        {
            for(j1 = 0; j1 < depth_image_milimeters_cv->image.cols; j1++)
            {
                const cv::Mat & original_image_ref_mm = depth_image_milimeters_cv->image;
                const cv::Mat & converted_image_ref_meters = depth_image_meters_cv.image;
                const cv::Mat & gray_image_ref = depth_gray_cv.image;
                value_original_mm_ptr = (uint16_t *) (original_image_ref_mm.data + original_image_ref_mm.step[0]*i1 + original_image_ref_mm.step[1]*j1); 
                value_converted_meters_ptr = (float *) (converted_image_ref_meters.data + converted_image_ref_meters.step[0] * i1 + converted_image_ref_meters.step[1] * j1);
                value_converted_gray_ptr = (uint8_t *) (gray_image_ref.data + gray_image_ref.step[0] * i1 + gray_image_ref.step[1] * j1);
                if (*value_original_mm_ptr == 0)
                {
                    *value_converted_meters_ptr = std::numeric_limits<float>::quiet_NaN();
                    *value_converted_gray_ptr = 0;
                }
                else
                {
                    *value_converted_meters_ptr = (float) (*value_original_mm_ptr)/1000.0;
                    if (*value_converted_meters_ptr > max_depth_value_meters)
                        max_depth_value_meters = *value_converted_meters_ptr;
                    if (*value_converted_meters_ptr < min_depth_value_meters)
                        min_depth_value_meters = *value_converted_meters_ptr; 
                    //*value_converted_gray_ptr = (uint8_t) floor((*value_converted_meters_ptr/DEPTH_MAX_VALUE_METERS)*255);
                }
            }
        }
        std::cout << "Max depth value: " << max_depth_value_meters << std::endl;
        std::cout << "Min depth value: " << min_depth_value_meters << std::endl;

        if (max_depth_value_meters > 10.0)
            max_depth_value_meters = 10;

        for(i1 = 0; i1 < depth_image_milimeters_cv->image.rows; i1++)
        {
            for(j1 = 0; j1 < depth_image_milimeters_cv->image.cols; j1++)
            {
                const cv::Mat & original_image_ref_mm = depth_image_milimeters_cv->image;
                const cv::Mat & converted_image_ref_meters = depth_image_meters_cv.image;
                const cv::Mat & gray_image_ref = depth_gray_cv.image;
                value_original_mm_ptr = (uint16_t *) (original_image_ref_mm.data + original_image_ref_mm.step[0]*i1 + original_image_ref_mm.step[1]*j1); 
                value_converted_meters_ptr = (float *) (converted_image_ref_meters.data + converted_image_ref_meters.step[0] * i1 + converted_image_ref_meters.step[1] * j1);
                value_converted_gray_ptr = (uint8_t *) (gray_image_ref.data + gray_image_ref.step[0] * i1 + gray_image_ref.step[1] * j1);
                if (*value_original_mm_ptr == 0)
                {
                    // Do nothing, because in the previous loop it has been assigned a 0 value in that case
                }
                else
                {
                    //*value_converted_gray_ptr = (uint8_t) floor((*value_converted_meters_ptr/DEPTH_MAX_VALUE_METERS)*255);
                    *value_converted_gray_ptr = (uint8_t) floor((*value_converted_meters_ptr-min_depth_value_meters)/(max_depth_value_meters - min_depth_value_meters)*255);
                }
            }
        }
        cv_bridge::CvImage gaussian_blur_image_cv;
        gaussian_blur_image_cv.encoding = sensor_msgs::image_encodings::MONO8;
        // Apply Blur
        if (use_gaussian_blur_)
        {
            cv::GaussianBlur(depth_gray_cv.image, gaussian_blur_image_cv.image, cv::Size(3,3), 2, 2);
            if (publish_debug_images_)
                gaussian_blur_image_pub_.publish(gaussian_blur_image_cv.toImageMsg());
        }
        else
        {
            gaussian_blur_image_cv.image = depth_gray_cv.image;
        }
        
        // Find fire circle using HoughCircles on color image
        std::vector<cv::Vec3f> circles_detected;
        if (publish_debug_images_)
        {
            cv_bridge::CvImage canny_edge_image;
            canny_edge_image.encoding = sensor_msgs::image_encodings::MONO8;
            cv::Canny(gaussian_blur_image_cv.image, canny_edge_image.image, canny_edge_upper_threshold_, canny_edge_upper_threshold_/2);
            canny_edge_image_pub_.publish(canny_edge_image.toImageMsg());
        }
        cv::HoughCircles(gaussian_blur_image_cv.image, circles_detected, CV_HOUGH_GRADIENT, dp_, min_dist_between_circle_center_, 
                         canny_edge_upper_threshold_, accu_th_, min_radius_pixels_, max_radius_pixels_);
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
            float circle_center_depth = (float)((depth_image_milimeters_cv->image.at<uint16_t>(center))/1000.0);
            std::cout << "Circle center depth: " << circle_center_depth << std::endl;
            // Calc 3D position of circle center
            geometry_msgs::Point circle_center_3D;
            get3DPointCameraModel(circle_center_3D, circle_center_depth, center.y, center.x);
            if (publish_debug_marker_)
            {
                circle_center_marker_.header.stamp = color_img_ptr->header.stamp;
                circle_center_marker_.pose.position = circle_center_3D;
                circle_center_marker_pub_.publish(circle_center_marker_);
            }
            object_detection_.header.stamp = color_img_ptr->header.stamp;
            object_detection_.header.frame_id = color_img_ptr->header.frame_id;
            object_detection_.pose.pose.position = circle_center_3D;
            object_detection_.pose.pose.orientation.w = 1.0; // TODO: Calc orientation
            object_detection_list_.stamp = color_img_ptr->header.stamp;
            object_detection_.scale.x = object_detection_.scale.y = object_detection_.scale.z = radius;
            // TODO: Fill covariance

            object_detection_list_.objects.push_back(object_detection_);
        }
        if (publish_debug_images_)
            detected_circles_image_pub_.publish(color_image_cv->toImageMsg());
        // Publish detected circles
        object_detection_pub_.publish(object_detection_list_);
        object_detection_list_.objects.clear();
    }
    else if (depth_img_ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        cv_bridge::CvImagePtr depth_image_meters_cv = cv_bridge::toCvCopy(depth_img_ptr, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_image_meters_cv->header = depth_img_ptr->header;
        depth_image_meters_cv->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        cv_bridge::CvImage depth_gray_cv;
        depth_gray_cv.image = cv::Mat(cv::Size(image_width, image_height), CV_8UC1);
        depth_gray_cv.header = depth_image_meters_cv->header;
        depth_gray_cv.encoding = sensor_msgs::image_encodings::MONO8;
        float * value_original_meters_ptr;
        uint8_t * value_converted_gray_ptr;
        int i1 = 0;
        int j1 = 0;
        // Convert depth image from milimeters to meters and also find min and max values
        for(i1 = 0; i1 < depth_image_meters_cv->image.rows; i1++)
        {
            for(j1 = 0; j1 < depth_image_meters_cv->image.cols; j1++)
            {
                const cv::Mat & original_image_ref_meters = depth_image_meters_cv->image;
                const cv::Mat & gray_image_ref = depth_gray_cv.image;
                value_original_meters_ptr = (float *) (original_image_ref_meters.data + original_image_ref_meters.step[0] * i1 + original_image_ref_meters.step[1] * j1);
                value_converted_gray_ptr = (uint8_t *) (gray_image_ref.data + gray_image_ref.step[0] * i1 + gray_image_ref.step[1] * j1);

                uint32_t integer_aux;
                std::memcpy(&integer_aux, value_original_meters_ptr, sizeof(float));
                if (integer_aux == 0x7FC00000)
                {
                    //std::cout << "NaN value detected" << std::endl;
                    //*value_original_meters_ptr = std::numeric_limits<float>::quiet_NaN();
                    *value_converted_gray_ptr = 0;
                }
                else
                {
                    if (*value_original_meters_ptr > max_depth_value_meters)
                        max_depth_value_meters = *value_original_meters_ptr;
                    if (*value_original_meters_ptr < min_depth_value_meters)
                        min_depth_value_meters = *value_original_meters_ptr; 
                }
            }
        }
        std::cout << "Max depth value: " << max_depth_value_meters << std::endl;
        std::cout << "Min depth value: " << min_depth_value_meters << std::endl;
        for(i1 = 0; i1 < depth_image_meters_cv->image.rows; i1++)
        {
            for(j1 = 0; j1 < depth_image_meters_cv->image.cols; j1++)
            {
                const cv::Mat & original_image_ref_meters = depth_image_meters_cv->image;
                const cv::Mat & converted_image_ref_meters = depth_image_meters_cv->image;
                const cv::Mat & gray_image_ref = depth_gray_cv.image;
                value_original_meters_ptr = (float *) (original_image_ref_meters.data + original_image_ref_meters.step[0]*i1 + original_image_ref_meters.step[1]*j1); 
                value_converted_gray_ptr = (uint8_t *) (gray_image_ref.data + gray_image_ref.step[0] * i1 + gray_image_ref.step[1] * j1);

                uint32_t integer_aux;
                std::memcpy(&integer_aux, value_original_meters_ptr, sizeof(float));
                if (integer_aux == 0x7FC00000)
                {
                    // Do nothing, because in the previous loop it has been assigned a 0 value in that case
                }
                else
                {
                    //*value_converted_gray_ptr = (uint8_t) floor((*value_converted_meters_ptr/DEPTH_MAX_VALUE_METERS)*255);
                    *value_converted_gray_ptr = (uint8_t) floor((*value_original_meters_ptr-min_depth_value_meters)/(max_depth_value_meters - min_depth_value_meters)*255);
                }
            }
        }
        // Apply Blur
        cv_bridge::CvImage gaussian_blur_image_cv;
        gaussian_blur_image_cv.encoding = sensor_msgs::image_encodings::MONO8;
        cv::GaussianBlur(depth_gray_cv.image, gaussian_blur_image_cv.image, cv::Size(3,3), 2, 2);
        if (publish_debug_images_)
            gaussian_blur_image_pub_.publish(gaussian_blur_image_cv.toImageMsg());
        // Find fire circle using HoughCircles on color image
        std::vector<cv::Vec3f> circles_detected;
        if (publish_debug_images_)
        {
            cv_bridge::CvImage canny_edge_image;
            canny_edge_image.encoding = sensor_msgs::image_encodings::MONO8;
            cv::Canny(gaussian_blur_image_cv.image, canny_edge_image.image, canny_edge_upper_threshold_, canny_edge_upper_threshold_/2);
            canny_edge_image_pub_.publish(canny_edge_image.toImageMsg());
        }
        cv::HoughCircles(gaussian_blur_image_cv.image, circles_detected, CV_HOUGH_GRADIENT, dp_, min_dist_between_circle_center_, 
                         canny_edge_upper_threshold_, accu_th_, min_radius_pixels_, max_radius_pixels_);
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
            float circle_center_depth = (float)(depth_image_meters_cv->image.at<float>(center));
            std::cout << "Circle center depth: " << circle_center_depth << std::endl;
            // Calc 3D position of circle center
            geometry_msgs::Point circle_center_3D;
            get3DPointCameraModel(circle_center_3D, circle_center_depth, center.y, center.x);
            if (publish_debug_marker_)
            {
                circle_center_marker_.header.stamp = color_img_ptr->header.stamp;
                circle_center_marker_.pose.position = circle_center_3D;
                circle_center_marker_pub_.publish(circle_center_marker_);
            }
            object_detection_.header.stamp = color_img_ptr->header.stamp;
            object_detection_.header.frame_id = color_img_ptr->header.frame_id;
            object_detection_.pose.pose.position = circle_center_3D;
            object_detection_.pose.pose.orientation.w = 1.0; // TODO: Calc orientation
            object_detection_list_.stamp = color_img_ptr->header.stamp;
            object_detection_.scale.x = object_detection_.scale.y = object_detection_.scale.z = radius;
            // TODO: Fill covariance
            
            object_detection_list_.objects.push_back(object_detection_);
        }
        if (publish_debug_images_)
            detected_circles_image_pub_.publish(color_image_cv->toImageMsg());
        // Publish detected circles
        object_detection_pub_.publish(object_detection_list_);
        object_detection_list_.objects.clear();
    }
    else
    {
        ROS_WARN("Invalid depth image encoding");
    }
    

}

void CircleDetector::findCirclesColor(const sensor_msgs::ImageConstPtr &color_img_ptr, const sensor_msgs::ImageConstPtr &depth_img_ptr)
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
    if (use_gaussian_blur_)
        cv::GaussianBlur(gray_image_cv.image, gaussian_blur_image_cv.image, cv::Size(gaussian_blur_kernel_size_, gaussian_blur_kernel_size_), gaussian_blur_sigma_, gaussian_blur_sigma_);
    else
        gaussian_blur_image_cv.image = gray_image_cv.image;
    
    if (publish_debug_images_)
        gaussian_blur_image_pub_.publish(gaussian_blur_image_cv.toImageMsg());
    // Find fire circle using HoughCircles on color image
    std::vector<cv::Vec3f> circles_detected;
    if (publish_debug_images_)
    {
        cv_bridge::CvImage canny_edge_image;
        canny_edge_image.encoding = sensor_msgs::image_encodings::MONO8;
        cv::Canny(gaussian_blur_image_cv.image, canny_edge_image.image, canny_edge_upper_threshold_, canny_edge_upper_threshold_/2);
        canny_edge_image_pub_.publish(canny_edge_image.toImageMsg());
    }
    cv::HoughCircles(gaussian_blur_image_cv.image, circles_detected, CV_HOUGH_GRADIENT, dp_, min_dist_between_circle_center_, 
                     canny_edge_upper_threshold_, accu_th_, min_radius_pixels_, max_radius_pixels_);
    for (size_t i = 0; i < circles_detected.size(); i++)
    {
        cv::Point center(cvRound(circles_detected[i][0]), cvRound(circles_detected[i][1]));
        int radius = cvRound(circles_detected[i][2]);
        std::cout << "Circle " << i << " radius : " << radius << std::endl;
        // Extract circle center depth from depth image
        float circle_center_depth = (float)((depth_image_cv->image.at<uint16_t>(center))/1000.0);
        std::cout << "Circle center depth: " << circle_center_depth << std::endl;

        if (circle_center_depth != 0 && circle_center_depth < 10.0 && circle_center_depth < max_depth_th_)
        {
            float circle_radius_meters = getRadiusInMetersFromPixels(circle_center_depth, radius);
            if ( (circle_radius_meters >= min_radius_meters_) && (circle_radius_meters <= max_radius_meters_ ) )
            {
                // circle center
                cv::circle( color_image_cv->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
                // circle outline
                cv::circle( color_image_cv->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
                std::cout << "Circle radius meters: " << circle_radius_meters << "\n";
                std::cout << "Circle center depth: " << circle_center_depth << std::endl;
                // Calc 3D position of circle center
                geometry_msgs::Point circle_center_3D;
                get3DPointCameraModel(circle_center_3D, circle_center_depth, center.y, center.x);
                if (publish_debug_marker_)
                {
                    circle_center_marker_.header.stamp = color_img_ptr->header.stamp;
                    circle_center_marker_.pose.position = circle_center_3D;
                    circle_center_marker_pub_.publish(circle_center_marker_);
                }
                object_detection_.header.stamp = color_img_ptr->header.stamp;
                object_detection_.header.frame_id = color_img_ptr->header.frame_id;
                object_detection_.pose.pose.position = circle_center_3D;
                //object_detection_.pose.pose.orientation.w = 1.0; // TODO: Calc orientation
                object_detection_list_.stamp = color_img_ptr->header.stamp;
                object_detection_.scale.x = object_detection_.scale.y = object_detection_.scale.z = radius;
                // TODO: Fill covariance

                object_detection_list_.objects.push_back(object_detection_);
            }
        }
    }
    if (publish_debug_images_)
        detected_circles_image_pub_.publish(color_image_cv->toImageMsg());
    // Publish detected circles
    object_detection_pub_.publish(object_detection_list_);
    object_detection_list_.objects.clear();
}

void CircleDetector::get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int &pixel_row, int &pixel_col)
{
    point.x = (pixel_col - model_.cx())/model_.fx() * depth;
    point.y = (pixel_row - model_.cy())/model_.fy() * depth;
    point.z = depth;
}

float CircleDetector::getRadiusInMetersFromPixels(float &depth, int &radius_pixel)
{
    return radius_pixel/( (model_.fx() + model_.fy())/2 ) * depth;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_detection_node");

    CircleDetector circle_detector;
    ros::spin();

    return 0;
}
