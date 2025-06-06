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

  bool get_point_cloud(const std::string & display_name,
      cv::OutputArrayOfArrays output_points,
      cv::OutputArrayOfArrays output_colors,
      cv::OutputArrayOfArrays output_mask,
      std::vector<std::vector<uint64_t>> * output_pids = nullptr) override;

protected:
  friend class c_ply_input_source;
  std::string filename_;
  std::vector<cv::Vec3f> points_;
  std::vector<cv::Vec3f> colors_;
};

#endif /* __c_ply_frame_h__ */
