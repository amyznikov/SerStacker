/*
 * vlo.h
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __vlo_h__
#define __vlo_h__

#include <core/io/c_vlo_file.h>

struct c_vlo_depth_segmentation_options
{
  double min_distance = 100; // [m]
  double max_distance = 300; // [m]
  double vlo_walk_error = 1; // [m]
  double min_slope = -0.3; // [tan = m / pix]
  double max_slope = +0.3; // [tan = m / pix]
  double min_height = 0.4; // [m]
  int counts_threshold= 500;
  int min_segment_size = 15;
};



bool vlo_depth_segmentation(const cv::Mat3f clouds[3],
    cv::Mat4f & output_histogram, cv::Mat3w & output_segments,
    const c_vlo_depth_segmentation_options & opts);

#endif /* __vlo_h__ */
