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


  bool get_data(DataViewType * selectedViewType,
      const std::string & channelName,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask) override;

  void set_image(const std::string & name, cv::InputArray image,
      cv::InputArray mask = cv::noArray());

  bool get_image(int id, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray()) ;

  bool get_image(const std::string & name, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray());

  void update_selection(cv::InputArray seletion_mask,
      SELECTION_MASK_MODE mode);

  void cleanup() override;

  void get_output_mask(cv::OutputArray output_mask);

protected:
  friend class c_image_input_source;
  cv::Mat input_image_, current_image_;
  cv::Mat1b input_mask_, current_mask_;
  std::map<std::string, cv::Mat> computed_images_;

  cv::Matx33f color_matrix_ = cv::Matx33f::eye();
  COLORID colorid_ = COLORID_UNKNOWN;
  int bpc_ = 0;
  bool has_color_matrix_ = false;
};

#endif /* __c_video_frame_h__ */
