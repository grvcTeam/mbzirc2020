#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "group_test");
   ros::NodeHandle _nh;
   ROS_INFO("[%s] Node initialization.", ros::this_node::getName().c_str());

   if (!ros::master::check())
   {
      ROS_ERROR("[%s] roscore is not running.", ros::this_node::getName().c_str());
      return EXIT_FAILURE;
   }

   std::vector<cv::Point2f> _candidates;
   _candidates.push_back(cv::Point2f(100, 100));
   _candidates.push_back(cv::Point2f(100, 110));
   _candidates.push_back(cv::Point2f(120, 111));
   _candidates.push_back(cv::Point2f(230, 10));
   _candidates.push_back(cv::Point2f(501, 643));
   _candidates.push_back(cv::Point2f(500, 250));
   _candidates.push_back(cv::Point2f(700, 900));
   _candidates.push_back(cv::Point2f(80, 100));
   _candidates.push_back(cv::Point2f(430, 210));
   _candidates.push_back(cv::Point2f(1000, 800));

   cv::Mat canvas(cv::Size(1200, 1000), CV_32FC3, cv::Scalar::all(0));

   for (auto point : _candidates)
   {
      cv::circle(canvas, point, 2, cv::Scalar(0, 0, 255), -1);
      cv::circle(canvas, point, 100, cv::Scalar(0, 0, 255), 1);
   }

   const int dist_max2 = 100 * 100;

   std::vector<cv::Point2f> centers = _candidates;

   ros::Time t0 = ros::Time::now();

   std::vector<int> groups;
   int groups_size = cv::partition(centers, groups, [dist_max2](const cv::Point2f& point1, const cv::Point2f& point2) {
      return (pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2)) < dist_max2;
   });

   ros::Duration duration = ros::Time::now() - t0;

   std::cout << "Took: " << duration.toSec() << " s" << std::endl;
   std::cout << "Group size: " << groups_size << std::endl;
   std::cout << "Groups: ";
   for (auto i : groups)
   {
      std::cout << i << ", ";
   }
   std::cout << std::endl;

   float sum_x[groups_size] = {0};
   float sum_y[groups_size] = {0};
   float count[groups_size] = {0};

   for (size_t j = 0; j < centers.size(); j++)
   {
      int index = groups[j];
      sum_x[index] += centers[j].x;
      sum_y[index] += centers[j].y;
      count[index]++;
   }

   cv::Point2f med[groups_size] = {cv::Point2f()};
   for (size_t k = 0; k < groups_size; k++)
   {
      med[k] = cv::Point2f(sum_x[k] / count[k], sum_y[k] / count[k]);
      cv::circle(canvas, med[k], 3, cv::Scalar(255, 0, 0), -1);
   }

   cv::imshow("Groups", canvas);
   cv::waitKey();

   ros::spin();

   ROS_INFO("[%s] Node finished.", ros::this_node::getName().c_str());

   return 0;
}