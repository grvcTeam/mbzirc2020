/*!
 * @file valid_detection.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 18/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */
#pragma once

#include <eigen3/Eigen/Geometry>

namespace mbzirc
{
class ValidDetection
{
  public:
   ValidDetection();
   ValidDetection(const float& x, const float& y, const float& depth, const float& sigma_xy, const float& sigma_z);
   ValidDetection(const Eigen::Vector2f center, const float& depth, Eigen::Matrix3f R);

   virtual ~ValidDetection();

   const Eigen::Vector2f& center() const;
   void center(const Eigen::Vector2f& center);

   const float& depth() const;
   void depth(const float& depth);

   const Eigen::Vector3d& local_position() const;
   void local_position(const Eigen::Vector3d& local_position);

   bool isSameDetection(const ValidDetection& detection, int max_dist = 100);

  private:
   Eigen::Vector2f _center;
   Eigen::Vector3d _local_position;

   float _depth;

   Eigen::Matrix3f _R;  // Covariance Matrix
};

}  // namespace mbzirc