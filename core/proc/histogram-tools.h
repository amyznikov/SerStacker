/*
 * histogram-tools.h
 *
 *  Created on: Mar 10, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __histogram_tools_h__
#define __histogram_tools_h__

#include "histogram.h"
#include <core/io/debayer.h>
#include <core/ctrlbind/ctrlbind.h>

enum histogram_normalization_type {
  normalize_histogram_mean,
  normalize_histogram_median,
  normalize_histogram_mode,
};

struct c_histogram_normalization_options
{
  histogram_normalization_type norm_type = normalize_histogram_mean;
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

/**
 *  dst = (src - mv) * stretch + offset
 *   => dst = src * stretch + offset - mv * stretch
 */
bool nomalizeImageHistogram(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst,
    const c_histogram_normalization_options & opts,
    enum COLORID src_colorid = COLORID_UNKNOWN);


/**
* @param qLow - lower quantile (e.g., 0.01 for 1%)
* @param qHigh - upper quantile (e.g., 0.99 for 99%)
*  */
bool histogramClipWhiteBalance(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst,
    double qlow, double qhigh);

#endif /* __histogram_tools_h__ */
