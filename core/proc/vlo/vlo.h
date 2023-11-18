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


enum vlo_depth_segmentation_output_type {
  vlo_depth_segmentation_output_segments,
  vlo_depth_segmentation_output_segment_distances,
  vlo_depth_segmentation_output_counts0,
  vlo_depth_segmentation_output_counts1,
};

struct c_vlo_depth_segmentation_options
{
  double min_distance = 100; // [cm]
  double max_distance = 30000; // [cm]
  double vlo_walk_error = 150; // [cm]
  double min_slope = -0.3; // [tan = cm / pix]
  double max_slope = +0.3; // [tan = cm / pix]
  int min_pts = 15;

  vlo_depth_segmentation_output_type output_type =
      vlo_depth_segmentation_output_segments;
};


bool vlo_depth_segmentation(cv::InputArray distances, cv::OutputArray output_data,
    const c_vlo_depth_segmentation_options & opts);


#endif /* __vlo_h__ */
