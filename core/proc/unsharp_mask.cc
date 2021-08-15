/*
 * unsharp_mask.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "unsharp_mask.h"



static void create_lpass_image(cv::InputArray src, cv::Mat & lpass, double sigma)
{

  constexpr int borderType = cv::BORDER_REFLECT;

  static const auto gaussian_blur =
      [](cv::InputArray src, cv::Mat & lpass, double sigma) {
        const cv::Mat1f G = cv::getGaussianKernel(2 * std::max(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
        cv::sepFilter2D(src, lpass, -1, G, G, cv::Point(-1, -1), 0, borderType);
      };

  int pyramid_level = 0;
  int Ci = 0;

  if( sigma > 2 ) {

    int min_size = std::min(src.cols(), src.rows());
    int imax = 0;
    while (min_size >>= 1) {
      ++imax;
    }

    const int C = (int) (sigma * sigma / 2);

    while (pyramid_level < imax && (1 + 4 * Ci) <= C) {
      Ci = 1 + 4 * Ci;
      ++pyramid_level;
    }
  }

  if ( pyramid_level < 1 ) {
    gaussian_blur(src, lpass, sigma);
    return;
  }


  std::vector<cv::Size> size_history;

  const double delta =
      sqrt(sigma * sigma - 2 * Ci) / (1 << pyramid_level)  ;


  size_history.emplace_back(src.size());
  cv::pyrDown(src, lpass, cv::Size(), borderType);

  for ( int j = 1; j < pyramid_level; ++j ) {
    size_history.emplace_back(lpass.size());
    cv::pyrDown(lpass, lpass, cv::Size(), borderType);
  }

  if ( delta > 0 ) {
    gaussian_blur(lpass, lpass, delta);
  }

  for ( int j = size_history.size()-1; j >= 0; --j ) {
    cv::pyrUp(lpass, lpass, size_history[j]);
  }
}


void unsharp_mask(cv::InputArray src, cv::OutputArray dst,
    double sigma, double alpha,
    double outmin, double outmax)
{

  if ( sigma <= 0 || alpha <= 0 ) {
    src.copyTo(dst);
  }
  else {

    if ( outmax <= outmin ) {

      const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

      switch ( ddepth ) {
      case CV_8U :
        outmin = 0, outmax = UINT8_MAX;
        break;
      case CV_8S :
        outmin = -INT8_MIN, outmax = INT8_MAX;
        break;
      case CV_16U :
        outmin = 0, outmax = UINT16_MAX;
        break;
      case CV_16S :
        outmin = INT16_MIN, outmax = INT16_MAX;
        break;
      case CV_32S :
        outmin = INT32_MIN, outmax = INT32_MAX;
        break;
      default : /*cv::minMaxLoc(src, &outmin, &outmax); */
        break;
      }
    }

    cv::Mat lpass;

#if 1 // totally faster but sligthly approximate
    create_lpass_image(src, lpass, sigma);
#else
    cv::Mat1f G = cv::getGaussianKernel(2 * std::max(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
    cv::sepFilter2D(src, lpass, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
#endif

    cv::addWeighted(src, 1. / (1. - alpha), lpass, -alpha / (1. - alpha), 0, dst);
  }

  if ( outmax > outmin ) {
    cv::min(dst.getMatRef(), outmax, dst.getMatRef());
    cv::max(dst.getMatRef(), outmin, dst.getMatRef());
  }
}


