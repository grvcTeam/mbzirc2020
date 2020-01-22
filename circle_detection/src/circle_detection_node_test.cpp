#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

class CircleDetector
{
public:
    CircleDetector();
    ~CircleDetector();
    void stats();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& image_ptr);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber color_image_sub_;
    image_transport::Publisher circles_raw_image_pub_;
    image_transport::Publisher circles_gaussian_image_pub_;
    image_transport::Publisher circles_median_image_pub_;
    image_transport::Publisher gray_scale_image_pub_;
    image_transport::Publisher gaussian_blur_image_pub_;
    image_transport::Publisher median_blur_image_pub_;
    image_transport::Publisher gray_edges_image_pub_;
    image_transport::Publisher gaussian_edges_image_pub_;
    image_transport::Publisher median_edges_image_pub_;

    cv_bridge::CvImagePtr received_color_image_;

    // Stats
    int grayscale_num_circles_detected_;
    int gaussian_blur_num_circles_detected_;
    int median_blur_num_circles_detected_;

    int grayscale_circle_avg_radius_;
    int gaussian_circle_avg_radius_;
    int median_circle_avg_radius_;
};

CircleDetector::CircleDetector():
nh_(),
pnh_("~"),
image_transport_(nh_),
grayscale_num_circles_detected_(0),
gaussian_blur_num_circles_detected_(0),
median_blur_num_circles_detected_(0),
grayscale_circle_avg_radius_(0),
gaussian_circle_avg_radius_(0),
median_circle_avg_radius_(0)
{
    color_image_sub_ = image_transport_.subscribe("image_topic", 1, &CircleDetector::imageCallback, this);
    circles_raw_image_pub_ = image_transport_.advertise("detected_circles_on_raw_image", 1);
    circles_gaussian_image_pub_ = image_transport_.advertise("detected_circles_on_gaussian_image", 1);
    circles_median_image_pub_ = image_transport_.advertise("detected_circles_on_median_image", 1);
    gray_scale_image_pub_ = image_transport_.advertise("gray_scale_image", 1);
    gaussian_blur_image_pub_ = image_transport_.advertise("gaussian_blur_image", 1);
    median_blur_image_pub_ = image_transport_.advertise("median_blur_image", 1);
    gray_edges_image_pub_ = image_transport_.advertise("gray_edges_image", 1);
    gaussian_edges_image_pub_ = image_transport_.advertise("gaussian_edges_image", 1);
    median_edges_image_pub_ = image_transport_.advertise("median_edges_image", 1);
}

CircleDetector::~CircleDetector()
{

}

void CircleDetector::imageCallback(const sensor_msgs::ImageConstPtr& image_ptr)
{
    try
    {
        received_color_image_ = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
        std::cout << "Received color image" << std::endl;
        std::cout << "Height: " << received_color_image_->image.rows << std::endl;
        std::cout << "Width: " <<  received_color_image_->image.cols << std::endl;
        std::cout << "Encoding: " <<  image_ptr->encoding << std::endl;
        std::cout << "Col step: " << received_color_image_->image.step[1] << std::endl;

        int image_height = received_color_image_->image.rows;
        int image_width = received_color_image_->image.cols;

        // Transform image to gray scale
        cv_bridge::CvImage gray_image_cv_bridge;
        //gray_image_cv_bridge.image = cv::Mat(cv::Size(image_width, image_height), CV_8U);
        gray_image_cv_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        cv::cvtColor(received_color_image_->image, gray_image_cv_bridge.image, CV_BGR2GRAY);

        // Reduce the noise so we avoid false circle detection
        cv_bridge::CvImage gaussian_blur_image_cv_bridge;
        gaussian_blur_image_cv_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        cv::GaussianBlur(gray_image_cv_bridge.image, gaussian_blur_image_cv_bridge.image, cv::Size(3,3), 2, 2);

        cv_bridge::CvImage median_blur_image_cv_bridge;
        median_blur_image_cv_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        cv::medianBlur(gray_image_cv_bridge.image, median_blur_image_cv_bridge.image, 3);

        std::vector<cv::Vec3f> circles_raw;
        std::vector<cv::Vec3f> circles_gaussian;
        std::vector<cv::Vec3f> circles_median;

        // Apply Canny edge detector to each one of the images (Raw, Gaussian Blur and Median Blur)
        cv_bridge::CvImage edges_raw_image;
        cv_bridge::CvImage edges_gaussian_image;
        cv_bridge::CvImage edges_median_image;
        edges_raw_image.encoding = edges_gaussian_image.encoding = edges_median_image.encoding = sensor_msgs::image_encodings::MONO8;

        double canny_edge_upper_threshold = 150;

        cv::Canny(gray_image_cv_bridge.image, edges_raw_image.image, canny_edge_upper_threshold, canny_edge_upper_threshold/2);
        cv::Canny(gaussian_blur_image_cv_bridge.image, edges_gaussian_image.image, canny_edge_upper_threshold, canny_edge_upper_threshold/2);
        cv::Canny(median_blur_image_cv_bridge.image, edges_median_image.image, canny_edge_upper_threshold, canny_edge_upper_threshold/2);

        // Apply the Hough Transform to find the circles
        int circles_min_radius = 35 ;
        int circles_max_radius = 51;
        int accumulator_threshold = 50;

        cv::HoughCircles(gray_image_cv_bridge.image, circles_raw, CV_HOUGH_GRADIENT, 1, gray_image_cv_bridge.image.rows/8, canny_edge_upper_threshold, accumulator_threshold, circles_min_radius, circles_max_radius);
        cv::HoughCircles(gaussian_blur_image_cv_bridge.image, circles_gaussian, CV_HOUGH_GRADIENT, 1, gray_image_cv_bridge.image.rows/8, canny_edge_upper_threshold, accumulator_threshold,circles_min_radius, circles_max_radius);
        cv::HoughCircles(median_blur_image_cv_bridge.image, circles_median, CV_HOUGH_GRADIENT, 1, gray_image_cv_bridge.image.rows/8, canny_edge_upper_threshold, accumulator_threshold, circles_min_radius, circles_max_radius);

        // Draw the circles detected
        cv_bridge::CvImagePtr circles_detected_image_grayscale;
        cv_bridge::CvImagePtr circles_detected_image_gaussian_blur;
        cv_bridge::CvImagePtr circles_detected_image_median_blur;
        circles_detected_image_grayscale = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
        circles_detected_image_gaussian_blur = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
        circles_detected_image_median_blur = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);

        //circles_detected_image_grayscale->encoding = circles_detected_image_gaussian_blur->encoding = circles_detected_image_median_blur->encoding = sensor_msgs::image_encodings::RGB8;

        grayscale_num_circles_detected_ += circles_raw.size();
        std::cout << "Circles Detected on Raw Grayscale image: " << circles_raw.size() << std::endl;
        for (size_t i = 0; i < circles_raw.size(); i++)
        {
            cv::Point center(cvRound(circles_raw[i][0]), cvRound(circles_raw[i][1]));
            int radius = cvRound(circles_raw[i][2]);
            std::cout << "Circle " << i << "radius " << radius << std::endl;
            grayscale_circle_avg_radius_ += radius;
            // circle center
            cv::circle( circles_detected_image_grayscale->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
            // circle outline
            cv::circle( circles_detected_image_grayscale->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
        }

        gaussian_blur_num_circles_detected_ += circles_gaussian.size();
        std::cout << "Circles Detected on Gaussian Blur image: " << circles_raw.size() << std::endl;
        for (size_t i = 0; i < circles_gaussian.size(); i++)
        {
            cv::Point center(cvRound(circles_gaussian[i][0]), cvRound(circles_gaussian[i][1]));
            int radius = cvRound(circles_gaussian[i][2]);
            std::cout << "Circle " << i << "radius " << radius << std::endl;
            gaussian_circle_avg_radius_ += radius;
            // circle center
            cv::circle( circles_detected_image_gaussian_blur->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
            // circle outline
            cv::circle( circles_detected_image_gaussian_blur->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
        }

        median_blur_num_circles_detected_ += circles_median.size();
        std::cout << "Circles Detected on Median Blur image: " << circles_median.size() << std::endl;
        for (size_t i = 0; i < circles_median.size(); i++)
        {
            cv::Point center(cvRound(circles_median[i][0]), cvRound(circles_median[i][1]));
            int radius = cvRound(circles_median[i][2]);
            std::cout << "Circle " << i << "radius " << radius << std::endl;
            median_circle_avg_radius_ += radius;
            // circle center
            cv::circle( circles_detected_image_median_blur->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
            // circle outline
            cv::circle( circles_detected_image_median_blur->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
        }

        std::cout << std::endl;

        gray_scale_image_pub_.publish(gray_image_cv_bridge.toImageMsg());
        gaussian_blur_image_pub_.publish(gaussian_blur_image_cv_bridge.toImageMsg());
        median_blur_image_pub_.publish(median_blur_image_cv_bridge.toImageMsg());
        circles_raw_image_pub_.publish(circles_detected_image_grayscale->toImageMsg());
        circles_gaussian_image_pub_.publish(circles_detected_image_gaussian_blur->toImageMsg());
        circles_median_image_pub_.publish(circles_detected_image_median_blur->toImageMsg());
        gray_edges_image_pub_.publish(edges_raw_image.toImageMsg());
        gaussian_edges_image_pub_.publish(edges_gaussian_image.toImageMsg());
        median_edges_image_pub_.publish(edges_median_image.toImageMsg());

    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_ptr->encoding.c_str());
    }
    
}

void CircleDetector::stats()
{
    std::cout << "Total stats: " << std::endl;
    std::cout << "Num circles detected with raw grayscale image: " << grayscale_num_circles_detected_;
    if (grayscale_num_circles_detected_ > 0)
        std::cout << " with avg radius: " << grayscale_circle_avg_radius_/grayscale_num_circles_detected_ << " pixels";
    std::cout << std::endl;
    std::cout << "Num circles detected with gaussian blur image: " << gaussian_blur_num_circles_detected_;
    if (gaussian_blur_num_circles_detected_ > 0)
        std::cout << " with avg radius: " << gaussian_circle_avg_radius_/gaussian_blur_num_circles_detected_ << " pixels";
    std::cout << std::endl;
    std::cout << "Num circles detected with median blur image: " << median_blur_num_circles_detected_;
    if (median_blur_num_circles_detected_ > 0)
        std::cout << " with avg radius: " << median_circle_avg_radius_/median_blur_num_circles_detected_ << " pixels";
    std::cout << std::endl;
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
    circle_detector.stats();

    return 0;
}