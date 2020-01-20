#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/ObjectDetection.h>

#include <iostream>

//static const std::string RECEIVED_IMAGE_WINDOW = "Depth image";
static const std::string CONVERTED_DEPTH_IMAGE_WINDOW = "Converted depth image";
static const std::string EDGES_IMAGE_WINDOW = "Edges image";
static const std::string HOUGH_LINES_IMAGE_WINDOW = "Hough Lines image";
static const std::string HOUGH_LINES_P_IMAGE_WINDOW = "Probabilistic Hough Lines image";
static const std::string CORNER_DETECTION_WINDOW = "Detected corners";

class WindowsDetector 
{
public:
    WindowsDetector();
    ~WindowsDetector();
private:
    void depthCallback(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::CameraInfoConstPtr &info_ptr);
    inline uint8_t mapFloatDepthImageToUInt8(float input);
    void detectWindows();
    void get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int pixel_row, int pixel_col);
    void bubbleSort(std::vector<cv::Point2f>& vector_point_pair);
    void swapPairs(std::vector<cv::Point2f>& vector_point_pair, int index_1, int index_2);
    bool checkWindowSize(geometry_msgs::Vector3& windowDimensions);
    float calcDistanceBetween3DPoints(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2);
    geometry_msgs::Point calcWindowCenter(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, 
                                          geometry_msgs::Point& point_3, geometry_msgs::Point& point_4);
    geometry_msgs::Vector3 normalizedVectorFrom2Points(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2);
    geometry_msgs::Quaternion calcWindowOrientation(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, 
                                                    geometry_msgs::Point& point_3, geometry_msgs::Point& point_4);
    geometry_msgs::Vector3 normalizedCrossProduct(geometry_msgs::Vector3& vector_1, geometry_msgs::Vector3& vector_2);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher corners_marker_pub_;
    ros::Publisher object_detection_pub_;
    ros::Publisher window_axis_markers_pub_;

    visualization_msgs::Marker corners_markers_msg_;
    visualization_msgs::Marker window_x_axis_marker_msg_;
    visualization_msgs::Marker window_y_axis_marker_msg_;
    visualization_msgs::Marker window_z_axis_marker_msg_;
    mbzirc_comm_objs::ObjectDetection object_detection_msg_;
    mbzirc_comm_objs::ObjectDetectionList object_detection_list_msg_;

    int image_counter_;

    image_geometry::PinholeCameraModel model_;

    cv_bridge::CvImage depth_image_8_;
    cv_bridge::CvImageConstPtr received_depth_image_opencv_ptr_;

    geometry_msgs::Point corner_upper_left_point_;
    geometry_msgs::Point corner_upper_right_point_;
    geometry_msgs::Point corner_lower_left_point_;
    geometry_msgs::Point corner_lower_right_point_;

    int maxCorners;
    cv::RNG rng;

    tf2_ros::TransformBroadcaster tf_broadcaster;
};
 
WindowsDetector::WindowsDetector():
image_transport_(nh_),
image_counter_(0),
rng(12345)
{
    image_transport::TransportHints hints("raw", ros::TransportHints(), ros::NodeHandle("~"));
    camera_sub_ = image_transport_.subscribeCamera("depth_image_topic", 1, &WindowsDetector::depthCallback, this, hints);
    image_pub_ = image_transport_.advertise("detected_lines_image", 1);
    corners_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("corners_markers", 1);
    object_detection_pub_ = nh_.advertise<mbzirc_comm_objs::ObjectDetectionList>("windows_detected", 1);
    window_axis_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("window_axis", 1);

    // Corner markers
    corners_markers_msg_.header.frame_id = "uav_1";
    corners_markers_msg_.id = 1;
    corners_markers_msg_.lifetime = ros::Duration(0);
    corners_markers_msg_.ns = "corners";
    corners_markers_msg_.type = visualization_msgs::Marker::SPHERE_LIST;
    corners_markers_msg_.action = visualization_msgs::Marker::ADD;
    corners_markers_msg_.pose.orientation.w = 1.0;
    corners_markers_msg_.scale.x = corners_markers_msg_.scale.y = corners_markers_msg_.scale.z = 0.2;
    corners_markers_msg_.color.a = 1.0;

    std_msgs::ColorRGBA color_upper_left, color_upper_right, color_lower_left, color_lower_right;
    color_upper_left.a = color_upper_right.a = color_lower_left.a = color_lower_right.a = 1.0;
    color_upper_left.r = 1.0; // Upper left corner RED
    color_upper_right.g = 1.0; // Upper right corner GREEN
    color_lower_left.r = color_lower_left.g = 1.0; // Lower left corner YELLOW
    color_lower_right.b = 1.0; // Lower right corner BLUE

    corners_markers_msg_.colors.push_back(color_upper_left);
    corners_markers_msg_.colors.push_back(color_upper_right);
    corners_markers_msg_.colors.push_back(color_lower_left);
    corners_markers_msg_.colors.push_back(color_lower_right);

    // Window axis markers initialization
    window_x_axis_marker_msg_.header.frame_id = "uav_1";
    window_x_axis_marker_msg_.ns = "window_axis";
    window_x_axis_marker_msg_.id = 0;
    window_y_axis_marker_msg_.header.frame_id = "uav_1";
    window_x_axis_marker_msg_.ns = "window_axis";
    window_y_axis_marker_msg_.id = 1;
    window_z_axis_marker_msg_.header.frame_id = "uav_1";
    window_z_axis_marker_msg_.ns = "window_axis";
    window_z_axis_marker_msg_.id = 2;
    window_x_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_y_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_z_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_x_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_y_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_z_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_x_axis_marker_msg_.scale.x = window_y_axis_marker_msg_.scale.x = window_z_axis_marker_msg_.scale.x = 0.01;
    window_x_axis_marker_msg_.scale.y = window_y_axis_marker_msg_.scale.y = window_z_axis_marker_msg_.scale.y = 0.02;
    window_x_axis_marker_msg_.color.a = window_y_axis_marker_msg_.color.a = window_z_axis_marker_msg_.color.a = 1.0;
    window_x_axis_marker_msg_.color.r = 1.0;
    window_y_axis_marker_msg_.color.g = 1.0;
    window_z_axis_marker_msg_.color.b = 1.0;

    // Object detection message initialization
    object_detection_msg_.header.frame_id = "uav_1";
    object_detection_msg_.type = object_detection_msg_.TYPE_PASSAGE;
    object_detection_msg_.color = object_detection_msg_.COLOR_UNKNOWN;

    // OpenCV Windows config
    //cv::namedWindow(RECEIVED_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(CONVERTED_DEPTH_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(EDGES_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(HOUGH_LINES_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(HOUGH_LINES_P_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
    //cv::moveWindow(RECEIVED_IMAGE_WINDOW, 250, 250);
    cv::moveWindow(EDGES_IMAGE_WINDOW, 250, 250);
    cv::moveWindow(HOUGH_LINES_IMAGE_WINDOW, 250, 250);
    cv::moveWindow(HOUGH_LINES_P_IMAGE_WINDOW, 250, 250);

    // Corner detection test
    cv::namedWindow(CORNER_DETECTION_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::moveWindow(CORNER_DETECTION_WINDOW, 250, 250);
}

WindowsDetector::~WindowsDetector()
{
    //cv::destroyWindow(RECEIVED_IMAGE_WINDOW);
    cv::destroyWindow(CONVERTED_DEPTH_IMAGE_WINDOW);
    cv::destroyWindow(EDGES_IMAGE_WINDOW);
    cv::destroyWindow(HOUGH_LINES_IMAGE_WINDOW);
    cv::destroyWindow(HOUGH_LINES_P_IMAGE_WINDOW);
}

void WindowsDetector::detectWindows()
{
    this->maxCorners = 4;

    /// Parameters for Shi-Tomasi algorithm
    std::vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    cv::Mat copy;
    copy = this->depth_image_8_.image.clone();

    cv::goodFeaturesToTrack( this->depth_image_8_.image,
                             corners,
                             this->maxCorners,
                             qualityLevel,
                             minDistance,
                             cv::Mat(),
                             blockSize,
                             useHarrisDetector,
                             k );

    std::cout << "Number of corners detected: " << corners.size() << std::endl;
    int r = 4;
    for ( int i = 0; i < corners.size(); i++)
    {
        cv::circle( copy, corners[i], r, cv::Scalar(this->rng.uniform(0,255), this->rng.uniform(0,255), this->rng.uniform(0,255)), -1, 8, 0);
        //std::cout << "Corner " << i << " : (x,y)" << corners[i].x << "," << corners[i].y << std::endl;
    }
    imshow(CORNER_DETECTION_WINDOW, copy);

    if (corners.size() == 4)
    {
        const cv::Mat & original_image_ref = this->received_depth_image_opencv_ptr_->image;
        original_image_ref.data + original_image_ref.step[0];
        int row,col;

        // TODO: Order corners so they appear in the required position of the vector
        this->bubbleSort(corners);

        // TODO: Instead of calculating 3D position of corner, calc position of a near pixel, because corner has a NaN value depth.
        // The selection of the near pixel depends on the corner position (i.e. upper left, upper right, ...).
        float *  value_original_ptr;
        // First corner: Upper Left
        row = corners[0].y - 1;
        col = corners[0].x - 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col ); 
        float corner_upper_left_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_upper_left_point_, corner_upper_left_depth, row, col);

        // Second corner: Lower Left
        row = corners[1].y + 1;
        col = corners[1].x - 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col );
        float corner_lower_left_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_lower_left_point_, corner_lower_left_depth, row, col);

        // Third corner: Upper Right
        row = corners[2].y - 1;
        col = corners[2].x + 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col ); 
        float corner_upper_right_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_upper_right_point_, corner_upper_right_depth, row, col);

        // Fourth corner: Lower Right
        row = corners[3].y + 1;
        col = corners[3].x + 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col );
        float corner_lower_right_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_lower_right_point_, corner_lower_right_depth, row, col);

        geometry_msgs::Vector3 window_dimensions;
        if (this->checkWindowSize(window_dimensions))
        {
            this->corners_markers_msg_.points.push_back(corner_upper_left_point_);
            this->corners_markers_msg_.points.push_back(corner_upper_right_point_);
            this->corners_markers_msg_.points.push_back(corner_lower_left_point_);
            this->corners_markers_msg_.points.push_back(corner_lower_right_point_);
            this->corners_markers_msg_.header.stamp = ros::Time::now();
            this->corners_marker_pub_.publish(this->corners_markers_msg_);
            this->corners_markers_msg_.points.clear();

            geometry_msgs::Point window_center = this->calcWindowCenter(corner_upper_left_point_, corner_lower_left_point_, 
                                                                        corner_upper_right_point_, corner_lower_right_point_);
            object_detection_msg_.pose.pose.position = window_center;
            geometry_msgs::Quaternion window_orientation = this->calcWindowOrientation(corner_upper_left_point_, corner_lower_left_point_,
                                                                                       corner_upper_right_point_, corner_lower_right_point_);
            object_detection_msg_.pose.pose.orientation = window_orientation;
            object_detection_msg_.scale = window_dimensions;

            // TODO: Publish tf2 transformation between uav_1 frame and window frame for debugging
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "uav_1";
            transformStamped.child_frame_id = "window";
            transformStamped.transform.rotation = window_orientation;
            transformStamped.transform.translation.x = window_center.x;
            transformStamped.transform.translation.y = window_center.y;
            transformStamped.transform.translation.z = window_center.z;
            tf_broadcaster.sendTransform(transformStamped);

            object_detection_list_msg_.objects.push_back(object_detection_msg_);
            this->object_detection_pub_.publish(object_detection_list_msg_);
            object_detection_list_msg_.objects.clear();
        }
    }
    else
    {
        std::cout << "Not enough corners detected" << std::endl;
    }
    
}

bool WindowsDetector::checkWindowSize(geometry_msgs::Vector3& window_dimensions)
{
    float distance_upper_left_upper_right = this->calcDistanceBetween3DPoints(this->corner_upper_left_point_, this->corner_upper_right_point_);
    float distance_lower_left_lower_right = this->calcDistanceBetween3DPoints(this->corner_lower_left_point_, this->corner_lower_right_point_);
    float distance_upper_left_lower_left = this->calcDistanceBetween3DPoints(this->corner_upper_left_point_, this->corner_lower_left_point_);
    float distance_upper_right_lower_right = this->calcDistanceBetween3DPoints(this->corner_upper_right_point_, this->corner_lower_right_point_);

    bool valid_distance_upper_left_upper_right = false;
    bool valid_distance_lower_left_lower_right = false;
    bool valid_distance_upper_left_lower_left = false;
    bool valid_distance_upper_right_lower_right = false;

    std::cout << "Distance between upper left and upper right corners: " << distance_upper_left_upper_right << std::endl;
    std::cout << "Distance between lower left and lower right corners: " << distance_lower_left_lower_right << std::endl;
    std::cout << "Distance between upper left and lower left corners: " << distance_upper_left_lower_left << std::endl;
    std::cout << "Distance between upper right and lower right corners: " << distance_upper_right_lower_right << std::endl;

    // Check window dimensions against a square of 2m x 2m
    if (distance_upper_left_upper_right > 1.9 && distance_upper_left_upper_right < 2.1)
    {
        valid_distance_upper_left_upper_right = true;
        window_dimensions.x = distance_upper_left_upper_right;
    }
    if (distance_lower_left_lower_right > 1.9 && distance_lower_left_lower_right < 2.1)
    {
        valid_distance_lower_left_lower_right = true;
    }
    if (distance_upper_left_lower_left > 1.9 && distance_upper_right_lower_right < 2.1)
    {
        valid_distance_upper_left_lower_left = true;
        window_dimensions.y = distance_upper_left_lower_left;
    }
    if (distance_upper_right_lower_right > 1.9 && distance_upper_right_lower_right < 2.1)
    {
        valid_distance_upper_right_lower_right = true;
    }

    // Check window dimensions against a rectangular window of 1.5m x 4m
    if (distance_upper_left_upper_right > 3.9 && distance_upper_left_upper_right < 4.1)
    {
        valid_distance_upper_left_upper_right = true;
        window_dimensions.x = distance_upper_left_upper_right;
    }
    if (distance_lower_left_lower_right > 3.9 && distance_lower_left_lower_right < 4.1)
    {
        valid_distance_lower_left_lower_right = true;
    }
    if (distance_upper_left_lower_left > 1.4 && distance_upper_right_lower_right < 1.6)
    {
        valid_distance_upper_left_lower_left = true;
        window_dimensions.y = distance_upper_left_lower_left;
    }
    if (distance_upper_right_lower_right > 1.4 && distance_upper_right_lower_right < 1.6)
    {
        valid_distance_upper_right_lower_right = true;
    }

    window_dimensions.z = 0;
    if (valid_distance_upper_left_upper_right && valid_distance_lower_left_lower_right && valid_distance_upper_left_lower_left && valid_distance_upper_right_lower_right)
    {
        std::cout << "Valid Window" << std::endl;
        return true;
    }
    else
    {
        std::cout << "Invalid Window" << std::endl;
        return false;
    }
}

geometry_msgs::Quaternion WindowsDetector::calcWindowOrientation(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, geometry_msgs::Point& point_3, geometry_msgs::Point& point_4)
{
    tf2::Quaternion tf2_quaternion;
    /*
        point_1 -> upper left corner
        point_2 -> lower left corner
        point_3 -> upper right corner
        point_4 -> lower right corner
    */
    geometry_msgs::Quaternion orientation;

    // 1st Get x,y,z axis of the window reference frame
    // The axis orientation will be similar to how the camera frame axis are oriented.
    // z perpendicular to the plane containing the corners
    // x parallel to the vector defined by upper left corner and upper right corner
    // y parallel to the vector defined by upper left corner and lower left corner
    geometry_msgs::Vector3 x_axis_window, y_axis_window, z_axis_window;

    x_axis_window = this->normalizedVectorFrom2Points(point_1, point_3);
    y_axis_window = this->normalizedVectorFrom2Points(point_1, point_2);
    z_axis_window = this->normalizedCrossProduct(x_axis_window, y_axis_window);

    // Publish axis markers (arrows) for debugging
    /*
    geometry_msgs::Point window_center = calcWindowCenter(point_1,point_2,point_3,point_4);

    
    geometry_msgs::Point end_point_x_axis, end_point_y_axis, end_point_z_axis;

    end_point_x_axis.x = window_center.x + x_axis_window.x;
    end_point_x_axis.y = window_center.y + x_axis_window.y;
    end_point_x_axis.z = window_center.z + x_axis_window.z;

    end_point_y_axis.x = window_center.x + y_axis_window.x;
    end_point_y_axis.y = window_center.y + y_axis_window.y;
    end_point_y_axis.z = window_center.z + y_axis_window.z;

    end_point_z_axis.x = window_center.x + z_axis_window.x;
    end_point_z_axis.y = window_center.y + z_axis_window.y;
    end_point_z_axis.z = window_center.z + z_axis_window.z;

    window_x_axis_marker_msg_.points.push_back(window_center);
    window_x_axis_marker_msg_.points.push_back(end_point_x_axis);
    window_y_axis_marker_msg_.points.push_back(window_center);
    window_y_axis_marker_msg_.points.push_back(end_point_y_axis);
    window_z_axis_marker_msg_.points.push_back(window_center);
    window_z_axis_marker_msg_.points.push_back(end_point_z_axis);

    window_axis_markers_pub_.publish(window_x_axis_marker_msg_);
    window_axis_markers_pub_.publish(window_y_axis_marker_msg_);
    window_axis_markers_pub_.publish(window_z_axis_marker_msg_);
    window_x_axis_marker_msg_.points.clear();
    window_y_axis_marker_msg_.points.clear();
    window_z_axis_marker_msg_.points.clear();
    */

    // Get M matrix which is the matrix that allows us to obtain window reference frame axis from camera referenc frame axis
    //     |a11 a12 a13|
    // M = |a21 a22 a23|
    //     |a31 a32 a33|
    // aij is the dot product of the i vector of the transformed basis and the j vector of the original basis
    // As the original basis is composed of the normal vectors î,ĵ and k (^), the dot product is simplified
    // and the resulting matrix elements are just the components of the transformed basis vectors, arranged by rows
    // The first vector in the first row, second vector in second row, ...
    double a11, a12, a13, a21, a22, a23, a31, a32, a33;

    a11 = x_axis_window.x;
    a12 = x_axis_window.y;
    a13 = x_axis_window.z;

    a21 = y_axis_window.x;
    a22 = y_axis_window.y;
    a23 = y_axis_window.z;

    a31 = z_axis_window.x;
    a32 = z_axis_window.y;
    a33 = z_axis_window.z;

    // Get M transpose (Rotation matrix that allows us to transform coordinates expressed in transformed basis to original basis)
    //      |a11 a21 a31|   |r11 r12 r13|
    // Mt = |a12 a22 a32| = |r21 r22 r23|
    //      |a13 a23 a33|   |r31 r32 r33|

    // Get yaw, pitch and roll from M transpose
    double yaw = atan2(a12, a11);
    double pitch = atan2(-a13, sqrt(a23*a23+a33*a33));
    double roll = atan2(a23, a33);

    // Get quaternion from roll,pitch and yaw
    tf2_quaternion.setRPY(roll, pitch, yaw);
    tf2::convert(tf2_quaternion, orientation);
    return orientation;
}

geometry_msgs::Vector3 WindowsDetector::normalizedVectorFrom2Points(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2)
{
    geometry_msgs::Vector3 vector;
    vector.x = point_2.x - point_1.x;
    vector.y = point_2.y - point_1.y;
    vector.z = point_2.z - point_1.z;

    double vector_norm = sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
    vector.x = vector.x/vector_norm;
    vector.y = vector.y/vector_norm;
    vector.z = vector.z/vector_norm;
    return vector;
}

geometry_msgs::Vector3 WindowsDetector::normalizedCrossProduct(geometry_msgs::Vector3& vector_1, geometry_msgs::Vector3& vector_2)
{
    geometry_msgs::Vector3 vector;
    vector.x = vector_1.y * vector_2.z - vector_1.z * vector_2.y;
    vector.y = - (vector_1.x * vector_2.z - vector_1.z * vector_2.x);
    vector.z = vector_1.x * vector_2.y - vector_1.y * vector_2.x;

    double vector_norm = sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
    vector.x = vector.x/vector_norm;
    vector.y = vector.y/vector_norm;
    vector.z = vector.z/vector_norm;
    return vector;
}

geometry_msgs::Point WindowsDetector::calcWindowCenter(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, geometry_msgs::Point& point_3, geometry_msgs::Point& point_4)
{
    geometry_msgs::Point center;

    center.x = (point_1.x + point_2.x + point_3.x + point_4.x)/4;
    center.y = (point_1.y + point_2.y + point_3.y + point_4.y)/4;
    center.z = (point_1.z + point_2.z + point_3.z + point_4.z)/4;

    return center;
}

float WindowsDetector::calcDistanceBetween3DPoints(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2)
{
    return sqrt( (point_1.x-point_2.x)*(point_1.x-point_2.x) + (point_1.y-point_2.y)*(point_1.y-point_2.y) + (point_1.z-point_2.z) );
}

void WindowsDetector::get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int pixel_row, int pixel_col)
{
    //std::cout << "Depth: " << depth << std::endl;
    //std::cout << "Pixel column: " << pixel_col << std::endl;
    //std::cout << "Pixel row: " << pixel_row << std::endl;
    point.x = (pixel_col - model_.cx())/model_.fx() * depth;
    point.y = (pixel_row - model_.cy())/model_.fy() * depth;
    point.z = depth;
}

void WindowsDetector::bubbleSort(std::vector<cv::Point2f>& vector_point_pair)
{
    int n = vector_point_pair.size();
    int i,j;
    bool swapped;

    // First order using x coordinate
    for (i=0; i < n-1; i++)
    {
        swapped = false;
        for (j=0; j < n-i-1; j++)
        {
            if (vector_point_pair[j].x > vector_point_pair[j+1].x)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }
    // Then order using y coordinate (half vector first, then the second half)
    for (i=0; i < n/2-1; i++)
    {
        swapped = false;
        for (j=0; j < n/2-i-1; j++)
        {
            if (vector_point_pair[j].y > vector_point_pair[j+1].y)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }
    for (i=0; i < n/2-1; i++)
    {
        swapped = false;
        for (j=n/2; j < n-i-1; j++)
        {
            if (vector_point_pair[j].y > vector_point_pair[j+1].y)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }    
}

void WindowsDetector::swapPairs(std::vector<cv::Point2f>& vector_point_pair, int index_1, int index_2)
{
    cv::Point2f temp;
    temp = vector_point_pair.at(index_1);
    vector_point_pair.at(index_1) = vector_point_pair.at(index_2);
    vector_point_pair.at(index_2) = temp;
}

#define DEPTH_MAX_VALUE 6
inline uint8_t WindowsDetector::mapFloatDepthImageToUInt8(float input)
{
    return floor((input/DEPTH_MAX_VALUE)*255);
}

void WindowsDetector::depthCallback(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::CameraInfoConstPtr &info_ptr)
{
    // For each 4 messages process 1 and discard the other 3
    if (image_counter_ == 0 || image_counter_ == 4)
    {
        image_counter_ = 0;
        model_.fromCameraInfo(info_ptr);
    
        try
        {
            //std::cout << "received image encoding " << image_ptr->encoding << std::endl;
            received_depth_image_opencv_ptr_ = cv_bridge::toCvShare(image_ptr, sensor_msgs::image_encodings::TYPE_32FC1);
            //std::cout << "opencv bridge image encoding " << received_depth_image_opencv_ptr_->encoding << std::endl;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        int image_height = image_ptr->height;
        int image_width = image_ptr->width;

        cv_bridge::CvImage depth_image_8, edges_image, hough_lines_image, hough_lines_p_image;
        depth_image_8.image = cv::Mat(cv::Size(image_width, image_height), CV_8UC1);
        depth_image_8.header.frame_id = image_ptr->header.frame_id;
        depth_image_8.header.stamp = image_ptr->header.stamp;

        edges_image.header.frame_id = image_ptr->header.frame_id;
        hough_lines_image.header.frame_id = image_ptr->header.frame_id;
        hough_lines_p_image.header.frame_id = image_ptr->header.frame_id;
        edges_image.header.stamp = image_ptr->header.stamp;
        hough_lines_image.header.stamp = image_ptr->header.stamp;
        hough_lines_p_image.header.stamp = image_ptr->header.stamp;
        edges_image.encoding = sensor_msgs::image_encodings::MONO8;
        hough_lines_image.encoding = sensor_msgs::image_encodings::BGR8;
        hough_lines_p_image.encoding = sensor_msgs::image_encodings::BGR8;

        for(int i1 = 0; i1 < received_depth_image_opencv_ptr_->image.rows; i1++)
        {
            for(int j1 = 0; j1 < received_depth_image_opencv_ptr_->image.cols; j1++)
            {
                const cv::Mat & original_image_ref = received_depth_image_opencv_ptr_->image;
                const cv::Mat & converted_image_ref = depth_image_8.image;
                float *  value_original_ptr = (float *) (original_image_ref.data + original_image_ref.step[0]*i1 + original_image_ref.step[1]*j1); 
                uint8_t * value_converted_ptr = (uint8_t *) (converted_image_ref.data + converted_image_ref.step[0] * i1 + converted_image_ref.step[1] * j1);

                uint32_t integer_aux;
                std::memcpy(&integer_aux, value_original_ptr, sizeof(float));
                //std::cout << std::hex << integer_aux << std::dec << ",";
                if (integer_aux == 0X7FC00000)
                {
                    *value_converted_ptr = 0;
                }
                else
                {
                    *value_converted_ptr = this->mapFloatDepthImageToUInt8(*value_original_ptr);
                } 
            }
        }
        std::cout << std::endl;

        depth_image_8_ = depth_image_8;

        cv::Canny(depth_image_8.image, edges_image.image, 50, 200, 3, true);
        cv::cvtColor(edges_image.image, hough_lines_image.image, CV_GRAY2BGR);
        cv::cvtColor(edges_image.image, hough_lines_p_image.image, CV_GRAY2BGR);

        std::vector<cv::Vec2f> lines;
        HoughLines(edges_image.image, lines, 1, CV_PI/180, 100, 0, 0);

        std::cout << "Hough Detected lines" << std::endl;
        for (size_t i=0; i < lines.size(); i++)
        {
            cv::Scalar color;
            switch (i%3)
            {
            case 0:
                color = cv::Scalar(0,0,255);
            case 1:
                color = cv::Scalar(0,255,0);
            case 2:
                color = cv::Scalar(255,0,0);
            default:
                break;
            }
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 2000*(-b));
            pt1.y = cvRound(y0 + 2000*(a));
            pt2.x = cvRound(x0 - 2000*(-b));
            pt2.y = cvRound(y0 - 2000*(a));

            std::cout << "Line " << i << " from: (" << pt1.x << "," << pt1.y << ") to " << "(" << pt2.x << "," << pt2.y << ")" << std::endl;
            std::cout << "rho: " << rho << " theta: " << theta << std::endl;
            std::cout << "Line " << i << " equation: y = " << -1/tan(theta) << "x + " << rho/sin(theta) << std::endl; 
            line( hough_lines_image.image, pt1, pt2, color, 3, CV_AA);
        }

        std::vector<cv::Vec4i> lines_p;
        HoughLinesP(edges_image.image, lines_p, 1, CV_PI/180, 50, 50, 100);
        std::cout << "Probabilistic Hough Detected lines" << std::endl;
        for (size_t i = 0; i < lines_p.size(); i++)
        {
           cv::Scalar color;
           switch (i%3)
           {
           case 0:
               color = cv::Scalar(0,0,255);
               break;
           case 1:
               color = cv::Scalar(0,255,0);
               break;
           case 2:
               color = cv::Scalar(255,0,0);
               break;
           default:
               break;
           }

           cv::Vec4i l_p = lines_p[i];
           line(hough_lines_p_image.image, cv::Point(l_p[0], l_p[1]), cv::Point(l_p[2], l_p[3]), color, 3, CV_AA);
           std::cout << "Line " << i << " from: (" << l_p[0] << "," << l_p[1] << ") to (" << l_p[2] << "," << l_p[3] << ")" << std::endl; 
        }

        //cv::imshow(RECEIVED_IMAGE_WINDOW, received_depth_image_opencv_ptr_->image);
        cv::imshow(CONVERTED_DEPTH_IMAGE_WINDOW, depth_image_8.image);
        cv::imshow(EDGES_IMAGE_WINDOW, edges_image.image);
        cv::imshow(HOUGH_LINES_IMAGE_WINDOW, hough_lines_image.image);
        cv::imshow(HOUGH_LINES_P_IMAGE_WINDOW, hough_lines_p_image.image);
        //cv::imshow(CORNER_DETECTION_WINDOW, depth_image_8_.image);
        this->detectWindows();
        image_pub_.publish(hough_lines_image.toImageMsg());

        //cv::waitKey();
    }
    image_counter_++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "windows_detector");
    WindowsDetector windows_detector;
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        cv::waitKey(1);
        rate.sleep();
    }

    return 0;
}