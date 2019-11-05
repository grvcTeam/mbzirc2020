/*!
 * @file camera_parameters.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 30/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>

namespace mbzirc
{
CameraParameters::CameraParameters()
{
   intrinsics.cx = 0.0;
   intrinsics.cy = 0.0;
   intrinsics.fx = 0.0;
   intrinsics.fy = 0.0;

   distortion.k1 = 0.0;
   distortion.k2 = 0.0;
   distortion.p1 = 0.0;
   distortion.p2 = 0.0;
   distortion.k3 = 0.0;
}

CameraParameters::~CameraParameters() {}
}  // namespace mbzirc