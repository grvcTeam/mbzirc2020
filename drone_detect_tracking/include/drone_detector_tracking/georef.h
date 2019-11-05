/*!
 * @file georef.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 22/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC

 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <eigen3/Eigen/Geometry>

#include <drone_detector_tracking/camera_calibration/camera_parameters.h>

namespace mbzirc
{
class Georef
{
  public:
   Georef(const CameraParameters& params, const Eigen::Isometry3d offset = Eigen::Isometry3d::Identity());
   virtual ~Georef();

   void updateBaseTf(const Eigen::Isometry3d& origin_pose);
   void updateBase2CameraTf(const Eigen::Isometry3d& offset);

   Eigen::Vector3d geolocalice(const float& x_px, const float& y_px, const float& depth,
                               Eigen::Vector3d& relative_pose);

  private:
   Intrinsics _intrinsic_params;

   Eigen::Isometry3d _base_tf;
   Eigen::Isometry3d _base_to_camera_tf;
   Eigen::Isometry3d _world2camera_tf;
};
}  // namespace mbzirc