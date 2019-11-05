/*!
 * @file distortion.h
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

namespace mbzirc
{
typedef struct Distortion
{
   double k1;
   double k2;
   double p1;
   double p2;
   double k3;
} Distortion;
}