//------------------------------------------------------------------------------
// GRVC MBZIRC
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------
#ifndef MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_GAZEBO_H
#define MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_GAZEBO_H

#include <mbzirc_components/magnetic_gripper.h>
#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>

class GazeboMagneticGripper: public MagneticGripper {
public:

    GazeboMagneticGripper(const std::string& _robot_name) {
        magnet_link_name_ = _robot_name + "::magnetic_gripper";

        ros::NodeHandle nh;
        std::string magnetize_advertise = _robot_name + "/magnetic_gripper/magnetize";
        magnetize_service_ = nh.advertiseService(magnetize_advertise, &GazeboMagneticGripper::magnetizeServiceCallback, this);

        std::string switch_pub_topic = _robot_name + "/catching_device/switch";
        switch_publisher_ = nh.advertise<std_msgs::Bool>(switch_pub_topic, 1);

        std::string link_states_sub_topic = "/gazebo/link_states";
        link_states_subscriber_ = nh.subscribe(link_states_sub_topic, 1, &GazeboMagneticGripper::linkStatesCallback, this);

        std::string link_state_pub_topic = "/gazebo/set_link_state";
        link_state_publisher_ = nh.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);

        pub_thread_ = std::thread([&]() {
            ros::Rate rate(100); // [Hz] 
            while (ros::ok()) {
                // Release object if magnet is not magnetized
                if (magnet_state_ != MagnetState::MAGNETIZED) {
                    grabbing_ = false;
                }
                if (grabbing_) {
                    double z_grabbing = -0.25;  // TODO: as a param? zero?
                    //std::cout << "Grabbing " << grabbed_link_name_ << std::endl;
                    gazebo_msgs::LinkState grabbed;
                    grabbed.link_name = grabbed_link_name_;
                    grabbed.pose.position.z = z_grabbing;
                    grabbed.reference_frame = magnet_link_name_;
                    link_state_publisher_.publish(grabbed);
                }
                // Publish switch state
                std_msgs::Bool switch_state;
                switch_state.data = grabbing_;
                switch_publisher_.publish(switch_state);
                // Sleep!
                rate.sleep();
            }
        });
    }

    // Ask about magnet status
    MagnetState magnetState() { return magnet_state_; }

    // Ask about switch status (won't give false positives)
    bool switchIsPressed() { return grabbing_; }

    // Handle the magnet
    void setMagnetization(bool magnetize) {
        magnet_state_ = (magnetize? MagnetState::MAGNETIZED : MagnetState::DEMAGNETIZED);
    }

    bool magnetizeServiceCallback(mbzirc_srvs::Magnetize::Request &_req, mbzirc_srvs::Magnetize::Response &_res) {
        setMagnetization(_req.magnetize);
        _res.success = true;
        return true;
    }

protected:
    MagnetState magnet_state_ = MagnetState::UNKNOWN;
    volatile bool grabbing_ = false;

    ros::Publisher switch_publisher_;
    ros::ServiceServer magnetize_service_;
    ros::Subscriber link_states_subscriber_;
    ros::Publisher link_state_publisher_;
    std::thread pub_thread_;

    std::string magnet_link_name_;
    std::string grabbed_link_name_;

    void linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr& _msg) {
        if (!grabbing_ && (magnet_state_ == MagnetState::MAGNETIZED)) {
            // All link states in world frame, find robot and grabbable objects
            Eigen::Vector3f robot_link_position;
            std::map<std::string, Eigen::Vector3f> name_to_position_map;
            for (size_t i = 0; i < _msg->name.size(); i++) {
                std::string link_name = _msg->name[i];
                if (link_name == magnet_link_name_) {
                    robot_link_position << _msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z;
                }
                std::size_t found_grabbable = link_name.find("grab_here");
                if (found_grabbable != std::string::npos) {
                    Eigen::Vector3f link_position(_msg->pose[i].position.x, \
                        _msg->pose[i].position.y, _msg->pose[i].position.z);
                    name_to_position_map[_msg->name[i]] = link_position;
                }
                // Calculate min distance between robot and all grabbable objects
                double min_distance = 1e3;  // Huge initial value (1km)
                for (auto& name_pos : name_to_position_map) {
                    Eigen::Vector3f diff = name_pos.second - robot_link_position;
                    double distance = diff.norm();
                    if (distance < min_distance) {
                        min_distance = distance;  // Keep min distance
                        grabbed_link_name_ = name_pos.first;  // and best candidate
                    }
                }
                // Check if min distance is below catching threshold TODO: as a param?
                double catching_threshold = 1.0;
                if (min_distance < catching_threshold) {
                    // Tell pub_thread_ to catch best candidate (grabbed_link_name_)
                    grabbing_ = true;
                }
            }
        }
    }
};

#endif  // MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_GAZEBO_H
