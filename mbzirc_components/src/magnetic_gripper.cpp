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
#include <mbzirc_components/magnetic_gripper.h>
#include <mbzirc_components/magnetic_gripper_serial.h>
#include <mbzirc_components/magnetic_gripper_gazebo.h>

MagneticGripper* MagneticGripper::createMagneticGripper(const std::string& _robot_name) {
    MagneticGripper* implementation = nullptr;
    bool use_sim_time;
    ros::param::param<bool>("use_sim_time", use_sim_time, false);
    if (use_sim_time) {
        implementation = new GazeboMagneticGripper(_robot_name);
    } else {
        implementation = new SerialMagneticGripper(_robot_name);
    }
    return implementation;
}
