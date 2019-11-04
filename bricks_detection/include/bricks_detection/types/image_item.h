/*!
 *      @file  image_Item.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  31/10/2019
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2019, FADA-CATEC
 */

#pragma once

#include <string>

#include <opencv2/core/core.hpp>

namespace mbzirc
{
class ImageItem
{
  public:
   ImageItem() {}
   virtual ~ImageItem(void) {}

   std::string color;

   cv::Point2i centroid;

   double area;
   double perimeter;
   double orientation;
};
}  // namespace mbzirc