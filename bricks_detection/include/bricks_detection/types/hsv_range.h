/*!
 *      @file  hsv_range.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  23/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <bricks_detection/types/hsv.h>

#include <string>

namespace mbzirc
{
class HSVRange
{
  public:
   HSVRange() { color_name = ""; }

   HSVRange(const HSV& min, const HSV& max, const std::string& color_name)
   {
      this->min        = min;
      this->max        = max;
      this->color_name = color_name;
   }

   HSV min;
   HSV max;
   std::string color_name;
};
}  // namespace mbzirc