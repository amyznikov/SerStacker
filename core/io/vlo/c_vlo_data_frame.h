/*
 * c_vlo_data_frame.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_frame_h__
#define __c_vlo_frame_h__

#include <core/io/vlo/c_vlo_file.h>
#include <core/io/c_data_frame.h>

class c_vlo_data_frame :
    public c_data_frame
{
public:
  typedef c_vlo_data_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  enum
  {
    ECHO0 = 0x1,
    ECHO1 = 0x2,
    ECHO2 = 0x4,
  };

  c_vlo_data_frame();

  bool get_data(DataViewType * selectedViewType,
      const std::string & channelName,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;

  void cleanup() override;

protected:
  void setup_default_channels();

public:
  c_vlo_scan current_scan_;
};

#endif /* __c_vlo_frame_h__ */
