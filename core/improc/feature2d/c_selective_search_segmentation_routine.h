/*
 * c_selective_search_segmentation_routine.h
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_selective_search_segmentation_routine_h__
#define __c_selective_search_segmentation_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/ximgproc/segmentation.hpp>

class c_selective_search_segmentation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_selective_search_segmentation_routine,
      "selective_search", "cv::ximgproc::segmentation::SelectiveSearchSegmentation");

  enum Strategy {
    Fast,
    Quality,
    Single
  };

  void set_strategy(Strategy v)
  {
    strategy_ = v;
    ss_.release();
  }

  Strategy strategy() const
  {
    return strategy_;
  }

  void set_base_k(int v)
  {
    base_k_ = v;
    ss_.release();
  }

  int base_k() const
  {
    return base_k_;
  }

  void set_inc_k(int v)
  {
    inc_k_ = v;
    ss_.release();
  }

  int inc_k() const
  {
    return inc_k_;
  }

  void set_sigma(float v)
  {
    sigma_ = v;
    ss_.release();
  }

  float sigma() const
  {
    return sigma_;
  }

  void set_max_display_rects(int v)
  {
    max_display_rects_ = v;
  }

  int max_display_rects() const
  {
    return max_display_rects_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation> ss_;
  Strategy strategy_ = Fast;
  int base_k_ = 150;
  int inc_k_ = 150;
  float sigma_ = 0.8f;

  int max_display_rects_ = 10;
  std::vector<cv::Rect> rects_;


};

#endif /* __c_selective_search_segmentation_routine_h__ */
