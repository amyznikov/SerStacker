/*
 * unkanala.h
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __unkanala_h__
#define __unkanala_h__

#include <opencv2/opencv.hpp>

struct c_kanala_intrinsics
{
  cv::Size image_size = cv::Size(3848, 2168);
  double focal_length_x = 1857;
  double focal_length_y = 1857;
  double principal_point_x = -1;
  double principal_point_y = -1;
  std::vector<double> distortion_coefficients; // must be 4 items
};

bool create_unkanala_remap(const c_kanala_intrinsics & intrinsics,
    cv::Mat2f & output_remap);



#endif /* __unkanala_h__ */
