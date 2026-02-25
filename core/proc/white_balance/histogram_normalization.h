/*
 * histogram_normalization.h
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __histogram_normalization_h__
#define __histogram_normalization_h__

#include <opencv2/opencv.hpp>
#include <core/io/debayer.h>
#include <core/ctrlbind/ctrlbind.h>

enum histogram_normalization_type {
  histogram_normalize_mean,
  histogram_normalize_median,
  histogram_normalize_mode
};

struct c_histogram_normalization_options
{
  histogram_normalization_type norm_type = histogram_normalize_median;
  cv::Scalar stretch = cv::Scalar(1, 1, 1, 1);
  cv::Scalar offset = cv::Scalar(0, 0, 0, 0);
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_histogram_normalization_options> & ctx)
{
  using S = c_histogram_normalization_options;
  ctlbind(ctls, "norm_type", ctx(&S::norm_type), "");
  ctlbind(ctls, "stretch", ctx(&S::stretch), "");
  ctlbind(ctls, "offset", ctx(&S::offset), "");
}

bool nomalize_image_histogramm(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst,
    histogram_normalization_type norm_type,
    const cv::Scalar & stretch = cv::Scalar(1, 1, 1, 1),
    const cv::Scalar & offset = cv::Scalar(0, 0, 0, 0),
    enum COLORID src_colorid = COLORID_UNKNOWN);

inline bool nomalize_image_histogramm(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst,
    const c_histogram_normalization_options & options,
    enum COLORID src_colorid = COLORID_UNKNOWN)
{
  return nomalize_image_histogramm(src, mask, dst,
      options.norm_type,
      options.stretch,
      options.offset,
      src_colorid);
}

#endif /* __histogram_normalization_h__ */
