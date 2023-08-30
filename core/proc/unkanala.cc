/*
 * unkanala.cc
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */


#include "unkanala.h"
#include <core/debug.h>

static const double hyp(double x, double y)
{
  return sqrt(x * x + y * y);
}

bool create_unkanala_remap(const c_kanala_intrinsics & intrinsics, cv::Mat2f & output_remap)
{
  const int w = intrinsics.image_size.width;
  const int h = intrinsics.image_size.height;

  if(  w < 1 || h < 1 ) {
    CF_ERROR("invalid image size not specified w=%d h=%d", w, h);
    return false;
  }

  const double fx = intrinsics.focal_length_x;
  const double fy = intrinsics.focal_length_y;
  if( fx <= 0 || fy <= 0 ) {
    CF_ERROR("invalid focal length specified: fx=%g fy=%g", fx, fy);
    return false;
  }


  const double cx =
      intrinsics.principal_point_x >= 0 ?
          intrinsics.principal_point_x :
          w / 2;

  const double cy =
      intrinsics.principal_point_y >= 0 ?
          intrinsics.principal_point_y :
          h / 2;

  output_remap.create(intrinsics.image_size);

  for ( uint yp = 0; yp < intrinsics.image_size.height; ++yp ) {
    for ( uint xp = 0; xp < intrinsics.image_size.width; ++xp ) {


      double td = atan(hyp((xp - cx), (yp - cy) * fx / fy) / fx);

      if ( intrinsics.distortion_coefficients.size() == 4 ) {

        const double & k1 = intrinsics.distortion_coefficients[0];
        const double & k2 = intrinsics.distortion_coefficients[1];
        const double & k3 = intrinsics.distortion_coefficients[2];
        const double & k4 = intrinsics.distortion_coefficients[3];

        const double t2 = td * td;
        const double t4 = t2 * t2;
        const double t6 = t4 * t2;
        const double t8 = t4 * t4;

        td *= (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
      }

      const double r = hyp(xp - cx, yp - cy);
      output_remap[yp][xp][0] = fx * td * (xp - cx) / r + cx;
      output_remap[yp][xp][1] = fy * td * (yp - cy) / r + cy;

    }
  }

  return true;
}

