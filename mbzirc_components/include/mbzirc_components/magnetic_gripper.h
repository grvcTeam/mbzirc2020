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
#ifndef MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_H
#define MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_H

#include <ros/ros.h>
#include <mbzirc_srvs/Magnetize.h>

class MagneticGripper {
public:
    enum class MagnetState { UNKNOWN, MAGNETIZED, DEMAGNETIZED };

    static MagneticGripper* createMagneticGripper(const std::string& _robot_name);

    // Ask about magnet status
    virtual MagnetState magnetState() = 0;

    // Ask about switch status
    virtual bool switchIsPressed() = 0;

    // Handle the magnet (may be up to ~4s blocking!)
    virtual void setMagnetization(bool _magnetize) = 0;

    virtual bool magnetizeServiceCallback(mbzirc_srvs::Magnetize::Request &_req,
        mbzirc_srvs::Magnetize::Response &_res) = 0;
};

#endif  // MBZIRC_COMPONENTS_MAGNETIC_GRIPPER_H
