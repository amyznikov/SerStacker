/*
 * smap.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "smap.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/eccalign.h>


#if 1
bool compute_smap(cv::InputArray src, cv::Mat & dst,
    int lksize, int scale_size, double minv)
{
  cv::Mat gs;
  //double noise;

  double mscale = 1.0;
  switch ( src.depth() ) {
  case CV_8U :
    case CV_8S :
    mscale = 1. / UINT8_MAX;
    break;
  case CV_16U :
    case CV_16S :
    mscale = 1. / UINT16_MAX;
    break;
  case CV_32S :
    mscale = 1. / UINT32_MAX;
    break;
  case CV_32F :
    case CV_64F :
    break;
  }

  if ( src.channels() == 1 ) {
    src.getMat().convertTo(gs, CV_32F, mscale);
  }
  else {
    cv::cvtColor(src, gs, cv::COLOR_BGR2GRAY);
    gs.convertTo(gs, CV_32F, mscale);
  }


  if ( lksize < 1 ) {
    lksize = 7;
  }

  if ( 1 ) {
    cv::Laplacian(gs, gs, CV_32F, lksize);
    cv::multiply(gs, gs, gs);
  }
  else  {
    cv::Mat gx, gy;
    cv::pyrDown(gs, gs);
    ecc_differentiate(gs, gx, gy, CV_32F);
    cv::magnitude(gx, gy, gs);
    cv::pyrUp(gs, gs, src.size());
  }

  if ( scale_size > 1 ) {
    ecc_downscale(gs, gs, scale_size, cv::BORDER_REPLICATE);
    ecc_upscale(gs, src.size());
  }

  if ( minv > 0 ) {
    cv::add(gs, minv, dst);
  }
  else {
    dst = std::move(gs);
  }

  return true;
}

#else
bool compute_smap(cv::InputArray src, cv::Mat & dst,
    double minv, double scale)
{
  cv::Mat gs, glap;

  double mscale = 1.0;
  switch ( src.depth() ) {
  case CV_8U :
    case CV_8S :
    mscale = 1. / UINT8_MAX;
    break;
  case CV_16U :
    case CV_16S :
    mscale = 1. / UINT16_MAX;
    break;
  case CV_32S :
    mscale = 1. / UINT32_MAX;
    break;
  case CV_32F :
    case CV_64F :
    break;
  }

  if ( src.channels() == 1 ) {
    src.getMat().convertTo(gs, CV_32F, mscale);
  }
  else {
    cv::cvtColor(src, gs, cv::COLOR_BGR2GRAY);
    gs.convertTo(gs, CV_32F, mscale);
  }


  const int num_levels = 3;
  std::vector<cv::Mat> glaps;
  glaps.reserve(num_levels);

  for ( int level = 0; level < num_levels; ++level ) {

    if ( level > 0 ) {
      cv::pyrDown(gs, gs);
    }

    static const cv::Mat1b SE(5, 5, 255);

    glaps.emplace_back();
    cv::morphologyEx(gs, glaps.back(), cv::MORPH_GRADIENT, SE);
    cv::multiply(glaps.back(), glaps.back(), glaps.back());
  }


  for ( int level = glaps.size() - 1; level > 0; --level ) {
    cv::pyrUp(glaps[level], glap, glaps[level - 1].size());
    cv::scaleAdd(glap, sqrt(2.), glaps[level - 1], glaps[level - 1]);
    //cv::scaleAdd(glap, 1 << level, glaps[level - 1], glaps[level - 1]);

  }

  if ( minv > 0 ) {
    cv::add(glaps.front(), minv, dst);
  }
  else {
    dst = std::move(glaps.front());
  }

  return true;
}
#endif

