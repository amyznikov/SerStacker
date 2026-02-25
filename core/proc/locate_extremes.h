/*
 * locate_extremes.h
 *
 *  Created on: Nov 21, 2023
 *      Author: amyznikov
 *
 * locate local extremal points which are local maximums or local minimums
 * among theirs neighborhood defined by given Structuring Element SE
 *
 */

#pragma once
#ifndef __locate_extremes_h__
#define __locate_extremes_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>


struct c_locate_extremes_options
{
  enum neighbor_filter_type
  {
    filter_morph,
    filter_mean,
  };

  neighbor_filter_type filter_type = filter_morph;
  cv::MorphShapes se_shape = cv::MORPH_RECT;
  cv::BorderTypes border_type = cv::BORDER_REPLICATE;
  cv::Scalar border_value;

  cv::Size se_size = cv::Size(3, 3);
  cv::Point anchor = cv::Point(-1, -1);

  bool locate_maximums = true;
  double maximums_alpha = 1;
  double maximums_beta = 0;

  bool locate_minimums = false;
  double minimums_alpha = 1;
  double minimums_beta = 0;
};

bool locate_extremes(cv::InputArray image, cv::InputArray input_mask,
    cv::OutputArray output_mask,
    const c_locate_extremes_options & opts);


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_locate_extremes_options> & ctx)
{
  using S = c_locate_extremes_options;
  ctlbind(ctls, "filter_type", ctx(&S::filter_type), "filter_type");
  ctlbind(ctls, "se_shape", ctx(&S::se_shape), "se_shape");
  ctlbind(ctls, "se_size", ctx(&S::se_size), "se_size");
  ctlbind(ctls, "anchor", ctx(&S::anchor), "anchor");
  ctlbind(ctls, "locate_maximums", ctx(&S::locate_maximums), "locate_maximums");
  ctlbind(ctls, "maximums_alpha", ctx(&S::maximums_alpha), "maximums_alpha");
  ctlbind(ctls, "maximums_beta", ctx(&S::maximums_beta), "maximums_beta");
  ctlbind(ctls, "locate_minimums", ctx(&S::locate_minimums), "locate_minimums");
  ctlbind(ctls, "minimums_alpha", ctx(&S::minimums_alpha), "minimums_alpha");
  ctlbind(ctls, "minimums_beta", ctx(&S::minimums_beta), "minimums_beta");
  ctlbind(ctls, "border_type", ctx(&S::border_type), "border_type");
  ctlbind(ctls, "border_value", ctx(&S::border_value), "border_value");
}



#endif /* __locate_extremes_h__ */
