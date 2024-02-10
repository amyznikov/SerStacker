/*
 * vlo_ghost_detection.h
 *
 *  Created on: Dec 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __vlo_ghost_detection_h__
#define __vlo_ghost_detection_h__

//#include <core/io/vlo/c_vlo_frame.h>
#include <core/io/vlo/c_vlo_scan.h>


struct c_vlo_ghost_detection_options
{
  double saturation_level = 122; // for 'peak' intensity measure
  double doubled_distanse_systematic_correction = 0; // [cm], -10 cm for makrolon, +40 cm for FIR20
  double doubled_distanse_depth_tolerance = 100; // [cm]

  bool drop_noise_behind_reflector = false;
};

bool vlo_ghost_detection(const c_vlo_scan & scan,
    const c_vlo_ghost_detection_options & opts,
    cv::Mat & output_ghsosts_mask);



#endif /* __vlo_ghost_detection_h__ */
