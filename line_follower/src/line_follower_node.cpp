//----------------------------------------------------------------------------------------------------------------------
// GRVC
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2020 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/ual_backend_dummy.h>
#include <ual_backend_mavros/ual_backend_mavros.h>
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>
#include <hue_object_detection/ImageConverter.h>

// TODO: Sliders for gains?
#define CONTROL_RATE 10
#define GRAY_BIN_THRESHOLD 128
#define DESIRED_HEIGHT 1.0
#define FORWARD_GAIN 0.75
#define LATERAL_GAIN 0.75
#define HEIGHT_GAIN 0.5
#define ROTATE_GAIN 0.5

int main(int argc, char** argv) {
  ros::init(argc, argv, "line_follower_node");

  std::string tf_prefix;
  std::string camera_url;
  std::string ual_backend;
  ros::param::param<std::string>("~tf_prefix", tf_prefix, "mbzirc_1");
  ros::param::param<std::string>("~camera_url", camera_url, "camera_0");
  ros::param::param<std::string>("~ual_backend", ual_backend, "mavros");

  grvc::ual::Backend *backend = nullptr;
  if (ual_backend == "dummy") {
    backend = new grvc::ual::BackendDummy();
  } else if (ual_backend == "mavros") {
    backend = new grvc::ual::BackendMavros();
  } else if (ual_backend == "gazebo_light") {
    backend = new grvc::ual::BackendGazeboLight();
  } else {
    throw std::runtime_error("Unexpected UAL backend");
  }
  grvc::ual::UAL ual(backend);

  do { sleep(1); } while (!ual.isReady());
  ual.takeOff(DESIRED_HEIGHT, true);

  ImageConverter image_converter(camera_url + "/camera_info", camera_url + "/image_raw", "line_detection", true);
  while (!image_converter.hasCameraInfo() && ros::ok()) {
    // TODO(performance): If camera info is constant, having a subscriber is overkill
    ROS_INFO("Waiting for camera info...");
    ros::spinOnce();
    sleep(1);
  }

  ros::Rate rate(CONTROL_RATE);  // [Hz]
  while (ros::ok()) {
    if (image_converter.hasNewImage()) {

      cv_bridge::CvImagePtr cv_ptr = image_converter.getCvImagePtr();
      auto frame = cv_ptr->image;
      if (frame.empty()) {
        ROS_WARN("Frame is empty!");
        continue;
      }

      cv::Mat gray, bin;
      cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      threshold(gray, bin, GRAY_BIN_THRESHOLD, 255, cv::THRESH_BINARY);

      auto frame_size = frame.size();
      int third_width  = cvFloor(frame_size.width  / 3);
      int third_height = cvRound(frame_size.height / 3);

      auto left_roi    = cv::Rect(            0, 0, third_width, third_height);
      auto central_roi = cv::Rect(  third_width, 0, third_width, third_height);
      auto right_roi   = cv::Rect(2*third_width, 0, third_width, third_height);

      double left_mean    = mean(bin(left_roi   ))[0];
      double central_mean = mean(bin(central_roi))[0];
      double right_mean   = mean(bin(right_roi  ))[0];

      rectangle(bin, left_roi, left_mean);
      rectangle(bin, central_roi, central_mean);
      rectangle(bin, right_roi, right_mean);
      // printf("%lf, %lf, %lf\n", left_mean, central_mean, right_mean);

      cv_ptr->image = bin;
      image_converter.publish(cv_ptr);  // TODO: Optional!

      geometry_msgs::TwistStamped velocity;
      velocity.header.frame_id = tf_prefix + "/base_link";
      velocity.header.stamp = ros::Time::now();
      velocity.twist.linear.x  = FORWARD_GAIN * (255 - central_mean) / 255.0;
      velocity.twist.linear.y  = LATERAL_GAIN * (right_mean - left_mean) / 255.0;
      velocity.twist.linear.z  = HEIGHT_GAIN  * (DESIRED_HEIGHT - ual.pose().pose.position.z);
      velocity.twist.angular.z = ROTATE_GAIN  * (right_mean - left_mean) / 255.0;

      // std::cout << velocity << '\n';
      ual.setVelocity(velocity);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
