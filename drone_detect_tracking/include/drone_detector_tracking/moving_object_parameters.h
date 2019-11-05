/*!
 * @file moving_object_parameters.h
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
class MovingObjectParameters
{
  public:
   MovingObjectParameters(){};

   int static_points_num;
   int dynamic_points_num;

   int lk_criteria_type;
   int lk_max_count;
   float lk_epsilon;

   float velocity_tolerance;
};

}  // namespace mbzirc