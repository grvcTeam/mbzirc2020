/*!
 *      @file  hsv.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/9/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

namespace mbzirc
{
class HSV
{
  public:
   HSV()
   {
      h = 0.f;
      s = 0.f;
      v = 0.f;
   }

   HSV(const float& h, const float& s, const float& v)
   {
      this->h = h;
      this->s = s;
      this->v = v;
   }

   float h;
   float s;
   float v;
};
}  // namespace mbzirc