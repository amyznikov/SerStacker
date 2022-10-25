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
#include <core/proc/morphology.h>

static cv::Mat1f create_dog_kernel(double _s1, double _s2)
{
  const double s1 =
      (std::min)(_s1, _s2);

  const double s2 =
      (std::max)(_s1, _s2);

  const int ksize =
      2 * (int) (4 * s2) + 1;

  const cv::Mat1f k1 =
      cv::getGaussianKernel(ksize, s1, CV_32F);

  const cv::Mat1f k2 =
      cv::getGaussianKernel(ksize, s2, CV_32F);

//  cv::Mat1f k;
//  cv::normalize(k1 - k2, k, 1, cv::NORM_L1);
//
//  return k;
  return k1 - k2;
}

static bool compute_dogsmap_(const cv::Mat & src, cv::Mat & dst, double s1, double s2, int scale, double minv)
{
  const cv::Size src_size =
      src.size();

  cv::Mat g, gx, gy;
  if ( s1 <= 0 ) {
    g = src;
  }
  else {
    const int gsize = 2 * (int) (4 * s1) + 1;
    const cv::Mat1f k1 = cv::getGaussianKernel(gsize, s1, CV_32F);
    cv::sepFilter2D(src, g, CV_32F, k1, k1.t());
  }

  ecc_differentiate(g, gx, gy);
  cv::magnitude(gx,  gy, g);
  cv::multiply(g, g, dst);


  if ( scale > 0 ) {
    ecc_downscale(dst, dst, scale, cv::BORDER_REPLICATE);
  }
  if ( minv > 0 ) {
    cv::add(dst, minv, dst);
  }
  if ( scale > 0 ) {
    ecc_upscale(dst, src_size);
  }

  return true;
}

bool compute_dogsmap(cv::InputArray src, cv::Mat & dst, double s1, double s2, int scale, double minv)
{
  cv::Mat gray;

  if( src.channels() == 1 ) {
    gray = src.getMat();
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  return compute_dogsmap_(gray, dst, s1, s2, scale, minv);
}


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

