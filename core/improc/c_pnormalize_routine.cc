/*
 * c_pnormalize_routine.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_pnormalize_routine.h"

void c_pnormalize_routine::pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale, PIXEL_DEPTH ddepth)
{
  cv::Mat m;
  cv::Scalar mean, stdev;
  double f = 0;

  if( m.channels() == 1 ) {
    pyramid_downscale(src, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
  }
  else {
    cv::cvtColor(src, m, cv::COLOR_BGR2GRAY);
    pyramid_downscale(m, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
    cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
  }

  cv::subtract(src, m, dst, cv::noArray(), ddepth);

//
//  cv::meanStdDev(m, mean, stdev);
//  for( int i = 0, cn = src.channels(); i < cn; ++i ) {
//    f += stdev[i];
//  }
//
//  m.convertTo(dst, CV_8U, 24. * src.channels() / f, 128);
}

