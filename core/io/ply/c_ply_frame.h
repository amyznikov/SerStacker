/*
 * c_ply_frame.h
 *
 *  Created on: Jan 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ply_frame_h__
#define __c_ply_frame_h__

#include <core/io/c_data_frame.h>

class c_ply_frame :
    public c_data_frame
{
public:
  typedef c_ply_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_ply_frame();

  std::string get_filename() override;

  bool get_display_data(DataViewType * selectedViewType,
      const std::string & channelName,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;

protected:
  friend class c_ply_input_source;
  std::string filename_;
  std::vector<cv::Vec3f> points_;
  std::vector<cv::Vec3f> colors_;
};

#endif /* __c_ply_frame_h__ */
