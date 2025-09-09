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
#include <core/io/hdl/c_hdl_specification.h>
#include <core/io/hdl/c_hdl_range_image.h>
#include <core/io/c_data_frame.h>

class c_hdl_data_frame :
    public c_data_frame
{
public:
  typedef c_hdl_data_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_hdl_data_frame();

  bool get_image(const std::string & display_name,
      cv::OutputArray output_image,
      cv::OutputArray output_mask,
      cv::OutputArray output_data ) final;

  bool get_point_cloud(const std::string & display_name,
      cv::OutputArray output_points,
      cv::OutputArray output_colors,
      cv::OutputArray output_mask,
      std::vector<uint64_t> * output_pids = nullptr) final;

  void clean_artifacts() final;

  void cleanup() final;

  c_hdl_range_image & range_image()
  {
    return range_image_;
  }

  const c_hdl_range_image & range_image() const
  {
    return range_image_;
  }

protected:
  void setup_default_channels();

protected:
  friend class c_hdl_input_source;
  c_hdl_frame::sptr current_frame_;
  c_hdl_specification current_lidar_;
  c_hdl_range_image range_image_;
  cv::Mat selection_mask_;
};

#endif /* __c_hdl_data_frame_h__ */
