/*!
 * @file reconfigure_parameters.h
 * @author Rafael Caballero González (RCG), rcaballero@catec.aero
 * @author Fidel González Leiva (FGL), fgonzalez@catec.aero
 *
 * @internal
 * Created 11/7/2019
 * Compiler gcc/g++
 * Company FADA-CATEC
 * Copyright (c) 2019, FADA-CATEC
 */

#pragma once

namespace mbzirc
{
class ReconfigureParameters
{
  public:
   ReconfigureParameters(){};

   int erosion_size;

   float unit_depth;
   float min_depth;
   float max_depth;
   float step_depth;

   int min_area;
   int max_area;

   float min_circularity;
   float max_circularity;

   float min_convexity;
   float max_convexity;

   int max_dist_local_detections;
   int max_dist_detections;
   int min_group_size;

   float sigma_xy;
   float sigma_z;

   int lk_criteria_type;
   int lk_max_count;
   float lk_epsilon;

   float velocity_tolerance;
};

class ColorReconfigureParameters
{
  public:
   ColorReconfigureParameters(){};

   int opening_size;
   int dilation_size;
   int blur_size;

   int h_upper_th;
   int h_lower_th;

   int s_upper_th;
   int s_lower_th;

   int v_upper_th;
   int v_lower_th;

   float min_circularity;
   float max_circularity;

   float ball_diameter;
};

}  // namespace mbzirc