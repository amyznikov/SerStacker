/*
 * c_pnormalize_routine.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_pnormalize_routine.h"

void c_pnormalize_routine::pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale)
{
  cv::Mat m;
  cv::Scalar mean, stdev;
  double f = 0;

  pyramid_downscale(src, m, pscale, cv::BORDER_REPLICATE);
  pyramid_upscale(m, src.size());
  m.convertTo(m, CV_32F);
  cv::subtract(src, m, m, cv::noArray(), CV_32F);

  cv::meanStdDev(m, mean, stdev);
  for( int i = 0, cn = src.channels(); i < cn; ++i ) {
    f += stdev[i];
  }

  m.convertTo(dst, CV_8U, 24. * src.channels() / f, 128);
}

