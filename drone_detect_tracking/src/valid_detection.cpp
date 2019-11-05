/*!
 * @file valid_detection.cpp
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 18/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#include <drone_detector_tracking/valid_detection.h>

namespace mbzirc
{
ValidDetection::ValidDetection() {}

ValidDetection::ValidDetection(const float& x, const float& y, const float& depth, const float& sigma_xy,
                               const float& sigma_z)
{
   this->_center = Eigen::Vector2f();
   this->_center << x, y;

   this->_depth = depth;

   // clang-format off
   this->_R = Eigen::Matrix3f();
   this->_R << sigma_xy,   0     , 0,
               0       , sigma_xy, 0,
               0       , 0       , sigma_z*std::pow(depth,2);
   // clang-format on
}

ValidDetection::ValidDetection(const Eigen::Vector2f center, const float& depth, Eigen::Matrix3f R)
{
   this->_center = center;
   this->_depth  = depth;
   this->_R      = R;
}

ValidDetection::~ValidDetection() {}

const Eigen::Vector2f& ValidDetection::center() const { return _center; }
void ValidDetection::center(const Eigen::Vector2f& center) { _center = center; }

const float& ValidDetection::depth() const { return _depth; }
void ValidDetection::depth(const float& depth) { _depth = depth; }

const Eigen::Vector3d& ValidDetection::local_position() const { return _local_position; }
void ValidDetection::local_position(const Eigen::Vector3d& local_position) { _local_position = local_position; }

bool ValidDetection::isSameDetection(const ValidDetection& detection, int max_dist) {}

}  // namespace mbzirc