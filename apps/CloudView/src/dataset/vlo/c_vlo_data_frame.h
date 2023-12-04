/*
 * c_vlo_data_frame.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_data_frame_h__
#define __c_vlo_data_frame_h__

#include "c_cloudview_data_frame.h"
#include <core/io/c_vlo_file.h>

namespace cloudview {

class c_vlo_data_frame :
    public c_cloudview_data_frame
{
public:
  typedef c_vlo_data_frame this_class;
  typedef c_cloudview_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  enum
  {
    AMBIENT_IMAGE,
    DISTANCES_IMAGE,
    DEPTH_IMAGE,
    ECHO_AREA_IMAGE,
    ECHO_PEAK_IMAGE,
    ECHO_WIDTH_IMAGE,

    DOUBLED_ECHO_DISTANCES_IMAGE,
    DOUBLED_ECHO_PEAKS_IMAGE,
    DOUBLED_ECHO_AREAS_IMAGE,
    GHOSTS_MASK_IMAGE,

    CLOUD3D,
    STRUCTURED_CLOUD3D,
  };

  c_vlo_data_frame();

  bool get_image(int id, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray());

  bool get_point_cloud(int id, cv::OutputArray points,
      cv::OutputArray colors);

public:
  c_vlo_scan current_scan_;
};

} /* namespace cloudview */

#endif /* __c_vlo_data_frame_h__ */
