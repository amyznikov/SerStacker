/*
 * vlo_bloom_detection.h
 *
 *  Created on: Jan 30, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __vlo_bloom_detection_h__
#define __vlo_bloom_detection_h__

#include <core/io/vlo/c_vlo_data_frame.h>


enum VLO_BLOOM_INTENSITY_MEASURE
{
  VLO_BLOOM_INTENSITY_PEAK,
  VLO_BLOOM_INTENSITY_AREA,
};

struct c_vlo_bloom_detection_options
{
  double intensity_saturation_level_020m = 145; // for 'peak' intensity measure
  double intensity_saturation_level_100m = 138; // for 'peak' intensity measure
  double bloom_min_intensity = 80;
  double bloom_intensity_tolerance = 7; // for 'peak' intensity measure
  double intensity_tolerance = 15; // for 'peak' intensity measure
  int max_reflector_hole_size = 11; // max hole size inside single reflector

  double min_bloom_slope = 0.01;
  double max_bloom_slope = 0.07;

  bool rreset = true;




  double min_distance = 100; // [cm]
  double max_distance = 30000;// [cm]
  double distance_tolerance = 150; // [cm]

  double min_segment_height = 15; // [pix], minimally acceptable vertical stdev of size of the wall in pixels
  double max_segment_slope = 20; // [deg]
  int min_segment_size = 15; // [points]
  int counts_threshold = 500; // normalized counter value


  VLO_BLOOM_INTENSITY_MEASURE intensity_measure = VLO_BLOOM_INTENSITY_AREA;
};

bool vlo_bloom_detection(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat & output_bloom_mask,
    cv::Mat & output_reflectors_mask,
    cv::Mat & output_debug_image);



#endif /* __vlo_bloom_detection_h__ */
