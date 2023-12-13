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
    AMBIENT = c_vlo_file::DATA_CHANNEL_AMBIENT,
    DISTANCES = c_vlo_file::DATA_CHANNEL_DISTANCES,
    DEPTH = c_vlo_file::DATA_CHANNEL_DEPTH,
    AREA = c_vlo_file::DATA_CHANNEL_AREA,
    PEAK = c_vlo_file::DATA_CHANNEL_PEAK,
    WIDTH = c_vlo_file::DATA_CHANNEL_WIDTH,
    GHOSTS = c_vlo_file::DATA_CHANNEL_GHOSTS_MASK,
  };

  c_vlo_data_frame();

  bool get_image(int id, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray());

  bool get_point_cloud(int id, cv::OutputArray points,
      cv::OutputArray colors);

  void getSupportedViewTypes(std::set<ViewType> * viewTypes) override;

  bool getViewData(ViewType * selectedViewType, int selectedDisplayId,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;


public:
  c_vlo_scan current_scan_;
};

} /* namespace cloudview */

#endif /* __c_vlo_data_frame_h__ */
