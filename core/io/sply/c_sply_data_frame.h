/*
 * c_sply_data_frame.h
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sply_data_frame_h__
#define __c_sply_data_frame_h__

#include <core/io/c_data_frame.h>

class c_sply_data_frame :
    public c_data_frame
{
public:
  typedef c_sply_data_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_sply_data_frame();

  bool get_point_cloud(const std::string & display_name,
      cv::OutputArrayOfArrays output_points,
      cv::OutputArrayOfArrays output_colors,
      cv::OutputArrayOfArrays output_masks) override;

protected:
  friend class c_sply_input_source;
  std::string filename_;

  std::vector<cv::Mat> _points;
  std::vector<cv::Mat> _colors;
  std::vector<double> _timestamps;

//  std::vector<cv::Vec3f> points_;
//  std::vector<cv::Vec3f> colors_;
//  std::vector<double> timestamps_;
};

#endif /* __c_sply_data_frame_h__ */
