/*
 * c_edge_preserving_filter_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_edge_preserving_filter_routine_h__
#define __c_edge_preserving_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_edge_preserving_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_edge_preserving_filter_routine,
      "edge_preserving_filter", "Apply cv::edgePreservingFilter(src, dst, filterType, sigma_s, sigma_r)");

  enum FilterType {
    RECURS_FILTER = cv::RECURS_FILTER, //!< Recursive Filtering
    NORMCONV_FILTER = cv::NORMCONV_FILTER //!< Normalized Convolution Filtering
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  FilterType filterType = RECURS_FILTER;
  float sigma_s = 60;
  float sigma_r = 0.4;
};

#endif /* __c_edge_preserving_filter_routine_h__ */
