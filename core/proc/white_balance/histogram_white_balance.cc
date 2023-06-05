/*
 * histogram_white_balance.cc
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#include "histogram_white_balance.h"
#include <core/proc/autoclip.h>
#include <core/debug.h>

bool histogram_white_balance(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst, double cl, double ch)
{
  if( src.channels() < 2 ) {
    src.copyTo(dst);
  }
  else {
    const int cn = src.channels();

    cv::Mat channels[cn];
    cv::Mat cmasks[cn];

    double imin[cn], imax[cn];
    int rc = 0;

    cv::split(src.getMat(), channels);

    if( mask.channels() > 1 ) {
      cv::split(mask.getMat(), cmasks);
    }
    else if( !mask.empty() && cv::countNonZero(mask) < mask.size().area() ) {
      for( int i = 0; i < cn; ++i ) {
        cmasks[i] = mask.getMat();
      }
    }

    for( int i = 0; i < cn; ++i ) {

      imin[i] = -1, imax[i] = -1;

      if( !compute_clip_levels(channels[i], cmasks[i], cl, ch, &imin[i], &imax[i]) ) {
        CF_ERROR("compute_clip_levels(channel=%d) fails", i);
        return false;
      }

      if( imax[i] - imin[i] > imax[rc] - imin[rc] ) {
        rc = i;
      }
    }

    for( int i = 0; i < cn; ++i ) {

      const double alpha = (imax[rc] - imin[rc]) / (imax[i] - imin[i]);
      const double beta = imin[rc] - alpha * imin[i];

      channels[i].convertTo(channels[i],
          channels[i].depth(),
          alpha, beta);
    }

    cv::merge(channels, cn, dst);
  }

  return true;

}
