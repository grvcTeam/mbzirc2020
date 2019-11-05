/*!
 * @file georef.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 22/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC

 * Copyright (c) 2019, FADA-CATEC
 */

#include <drone_detector_tracking/georef.h>

namespace mbzirc
{
Georef::Georef(const CameraParameters& params, const Eigen::Isometry3d offset) : _base_tf(Eigen::Isometry3d::Identity())
{
   _intrinsic_params  = params.intrinsics;
   _base_to_camera_tf = offset;
}

Georef::~Georef() {}

void Georef::updateBaseTf(const Eigen::Isometry3d& origin_pose)
{
   _base_tf         = origin_pose;
   _world2camera_tf = _base_tf * _base_to_camera_tf;
}
void Georef::updateBase2CameraTf(const Eigen::Isometry3d& offset)
{
   _base_to_camera_tf = offset;
   _world2camera_tf   = _base_tf * _base_to_camera_tf;
}

Eigen::Vector3d Georef::geolocalice(const float& x_px, const float& y_px, const float& depth,
                                    Eigen::Vector3d& relative_pose)
{
   double x = depth * ((double)x_px - _intrinsic_params.cx) / _intrinsic_params.fx;
   double y = depth * ((double)y_px - _intrinsic_params.cy) / _intrinsic_params.fy;
   double z = depth;

   Eigen::Isometry3d detection_relative_pose;
   detection_relative_pose = Eigen::Translation3d(z, -x, -y);  // FLU -> RDF (x-Right, y-Down, z-Forward)

   Eigen::Vector3d detection_absolute_position = (_world2camera_tf * detection_relative_pose).translation();

   relative_pose = detection_relative_pose.translation();

   return detection_absolute_position;
}
}  // namespace mbzirc