/*
 * c_video_frame.h
 *
 *  Created on: Nov 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_video_frame_h__
#define __c_cloudview_video_frame_h__

#include "c_cloudview_data_frame.h"
#include <core/io/debayer.h> // for COLORID

namespace cloudview {

class c_video_frame :
    public c_cloudview_data_frame
{
public:
  typedef c_video_frame this_class;
  typedef c_cloudview_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_video_frame();

public:
  cv::Mat image;
  cv::Mat1b mask;
  cv::Matx33f color_matrix = cv::Matx33f::eye();

  COLORID colorid = COLORID_UNKNOWN;
  int bpc = 0;
  bool has_color_matrix = false;

};

} /* namespace cloudview */

#endif /* __c_cloudview_video_frame_h__ */
