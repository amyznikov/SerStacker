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
    _strategy = v;
    _ss.release();
  }

  Strategy strategy() const
  {
    return _strategy;
  }

  void set_base_k(int v)
  {
    _base_k = v;
    _ss.release();
  }

  int base_k() const
  {
    return _base_k;
  }

  void set_inc_k(int v)
  {
    _inc_k = v;
    _ss.release();
  }

  int inc_k() const
  {
    return _inc_k;
  }

  void set_sigma(float v)
  {
    _sigma = v;
    _ss.release();
  }

  float sigma() const
  {
    return _sigma;
  }

  void set_max_display_rects(int v)
  {
    _max_display_rects = v;
  }

  int max_display_rects() const
  {
    return _max_display_rects;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Ptr<cv::ximgproc::segmentation::SelectiveSearchSegmentation> _ss;
  Strategy _strategy = Fast;
  int _base_k = 150;
  int _inc_k = 150;
  float _sigma = 0.8f;

  int _max_display_rects = 10;
  std::vector<cv::Rect> _rects;


};

#endif /* __c_selective_search_segmentation_routine_h__ */
