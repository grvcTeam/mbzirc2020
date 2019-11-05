/*!
 * @file camera_parameters.h
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

#include <drone_detector_tracking/camera_calibration/distortion.h>
#include <drone_detector_tracking/camera_calibration/intrinsics.h>

namespace mbzirc
{
class CameraParameters
{
  public:
   CameraParameters();
   virtual ~CameraParameters();

   Intrinsics intrinsics;
   Distortion distortion;
};
}  // namespace mbzirc