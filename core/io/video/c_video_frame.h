/*
 * c_video_frame.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_video_frame_h__
#define __c_video_frame_h__

#include <core/io/c_data_frame.h>
#include <core/io/debayer.h> // for COLORID

class c_video_frame :
    public c_data_frame
{
public:
  typedef c_video_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  enum
  {
    PIXEL_VALUE = 0,
  };

  c_video_frame();

  bool get_display_data(DataViewType * selectedViewType, int selectedDisplayId,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;

  bool get_image(int id, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray()) override;

public:
  cv::Mat image;
  cv::Mat1b mask;
  cv::Matx33f color_matrix = cv::Matx33f::eye();
  COLORID colorid = COLORID_UNKNOWN;
  int bpc = 0;
  bool has_color_matrix = false;
};

#endif /* __c_video_frame_h__ */
