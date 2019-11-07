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
   ImageItem(std::string color)
   {
      this->color = color;

      if (color == "red")
         color_id = 1;
      else if (color == "green")
         color_id = 2;
      else if (color == "blue")
         color_id = 3;
      else if (color == "orange")
         color_id = 4;
      else
         color_id = 0;
   }
   virtual ~ImageItem(void) {}

   std::string color;
   uint8_t color_id;

   cv::Point2i centroid;

   double area;
   double perimeter;
   double orientation;
};
}  // namespace mbzirc