/*
 * c_hdl_data_frame.h
 *
 *  Created on: Feb 25, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_data_frame_h__
#define __c_hdl_data_frame_h__

#include <core/io/hdl/c_hdl_frame.h>
#include <core/io/c_data_frame.h>

class c_hdl_data_frame :
    public c_data_frame
{
public:
  typedef c_hdl_data_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_hdl_data_frame();

  bool get_data(DataViewType * selectedViewType,
      const std::string & channelName,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;

  void cleanup() override;

protected:
  void setup_default_channels();

public:
  c_hdl_frame::sptr current_frame_;
  cv::Mat selection_mask_;
};

#endif /* __c_hdl_data_frame_h__ */
