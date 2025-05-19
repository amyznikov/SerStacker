/*
 * fast_gaussian_blur.cc
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#include "fast_gaussian_blur.h"

static void create_lpass_image(cv::InputArray src, cv::Mat & lpass, double sigma, double delta, double scale, int borderType, int ddepth)
{
  static const auto gaussian_blur =
      [](cv::InputArray src, cv::Mat & lpass, double sigma, double delta, double scale, int borderType, int ddepth) {
        const cv::Mat1f G = cv::getGaussianKernel(2 * (std::max)(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
        cv::sepFilter2D(src, lpass, ddepth, G * scale, G, cv::Point(-1, -1), delta, borderType);
      };

  int pyramid_level = 0;
  int Ci = 0;

  if( sigma > 2 ) {

    int min_size =
        (std::min)(src.rows(),
            src.cols());

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
    gaussian_blur(src, lpass, sigma, delta, scale, borderType, ddepth);
    return;
  }


  std::vector<cv::Size> size_history;

  const double ds =
      sqrt(sigma * sigma - 2 * Ci) / (1 << pyramid_level)  ;


  size_history.emplace_back(src.size());
  cv::pyrDown(src, lpass, cv::Size(), borderType);

  for ( int j = 1; j < pyramid_level; ++j ) {
    size_history.emplace_back(lpass.size());
    cv::pyrDown(lpass, lpass, cv::Size(), borderType);
  }

  if ( ds > 0 ) {
    gaussian_blur(lpass, lpass, ds, delta, scale, borderType, ddepth);
  }

  for( int j = size_history.size() - 1; j >= 0; --j ) {
    cv::pyrUp(lpass, lpass, size_history[j]);
  }
}


// When mask is not empty the src pixels MUST be also set to 0 where _mask is 0
void fast_gaussian_blur(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    double sigma, int borderType, int ddepth)
{
  if ( _mask.empty() || cv::countNonZero(_mask) == _mask.size().area() ) {

    cv::Mat lpass;
    create_lpass_image(_src, lpass, sigma, 0, 1, borderType, ddepth);
    _dst.move(lpass);
  }
  else {

    cv::Mat gsrc, gmask;

    create_lpass_image(_src, gsrc, sigma, 0, 1, borderType, CV_32F);
    create_lpass_image(_mask, gmask, sigma, 1e-12, 1. / 255, borderType, CV_32F);

    if( gsrc.channels() != gmask.channels() ) {
      cv::merge(std::vector<cv::Mat>(gsrc.channels(), gmask), gmask);
    }

    cv::divide(gsrc, gmask, _dst, 1, ddepth);
  }
}
