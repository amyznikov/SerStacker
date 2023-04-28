/*
 * c_gaussian_filter.cc
 *
 *  Created on: Aug 18, 2021
 *      Author: amyznikov
 */

#include "c_gaussian_filter.h"
#include <core/debug.h>

c_gaussian_filter::c_gaussian_filter()
{
}

c_gaussian_filter::c_gaussian_filter(double sigmaX, double sigmaY, const cv::Size & ksize, double scale) :
  sigmaX_(sigmaX), sigmaY_(sigmaY), ksize_(ksize), scale_(scale)
{
  create_gaussian_kernels(Kx_, Ky_, CV_32F, ksize_, sigmaX_, sigmaY_, scale_);
}

double c_gaussian_filter::sigmax() const
{
  return sigmaX_;
}

double c_gaussian_filter::sigmay() const
{
  return sigmaY_;
}

void c_gaussian_filter::create_gaussian_kernels(cv::Mat & kx, cv::Mat & ky, int ktype, cv::Size & ksize, double sigmax, double sigmay, double scale)
{
  const int kdepth =
      CV_MAT_DEPTH(ktype);

  // Automatic detection of kernel size from sigma if not specified

  if( sigmax < 0 ) {
    sigmax = sigmay > 0 ? sigmay : 0;
  }
  if( sigmay < 0 ) {
    sigmay = sigmax > 0 ? sigmax : 0;
  }

  if ( ksize.width <= 0 ) {
    if ( sigmax > 0 ) {
      ksize.width = cvRound(sigmax * (kdepth == CV_8U ? 3 : 4) * 2 + 1) | 1;
    }
    else {
      ksize.width = 1;
    }
  }

  if ( ksize.height <= 0 ) {
    if ( sigmay > 0  ) {
      ksize.height = cvRound(sigmay * (kdepth == CV_8U ? 3 : 4) * 2 + 1) | 1;
    }
    else {
      ksize.height = 1;
    }
  }

  CV_Assert(ksize.width > 0 && ksize.width % 2 == 1 && ksize.height > 0 && ksize.height % 2 == 1);

  if( sigmax > 0 ) {
    kx = cv::getGaussianKernel(ksize.width, sigmax,
        std::max(kdepth, CV_32F)) * scale;
  }
  else {
    kx = cv::Mat1f::ones(1, 1) * scale;
  }

  if ( sigmay > 0 ) {
    ky = cv::getGaussianKernel(ksize.height, sigmay,
        std::max(kdepth, CV_32F)) * scale;
  }
  else {
    ky = cv::Mat1f::ones(1, 1) * scale;
  }
}

void c_gaussian_filter::apply(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst, int borderType, int ddepth) const
{
  const cv::Size src_size =
      _src.size();

  const int src_type =
      _src.type();

  if ( _dst.fixedType() ) {
    ddepth =
        _dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth =
        _src.depth();
  }

  if ( Kx_.rows == 1 &&  Ky_.rows == 1 ) {

    _src.getMat().convertTo(_dst, ddepth);

    return;
  }

  if ( _mask.empty() || cv::countNonZero(_mask) == _mask.size().area() ) {

    cv::sepFilter2D(_src, _dst,
        ddepth,
        Kx_, Ky_,
        cv::Point(-1, -1),
        0,
        borderType);

    return;
  }

  cv::Mat gsrc, gmask;

  cv::sepFilter2D(_src, gsrc,
      CV_32F,
      Kx_, Ky_,
      cv::Point(-1, -1),
      0,
      borderType);

  cv::sepFilter2D(_mask, gmask,
      CV_32F,
      (1.0 / 255) * Kx_, Ky_,
      cv::Point(-1, -1),
      1e-12, // set to some small number to prevent division by zero below
      borderType);

  const cv::Mat1b src_mask =
      _mask.getMat();

  if( gsrc.channels() != gmask.channels() ) {
    cv::merge(std::vector<cv::Mat>(gsrc.channels(), gmask), gmask);
  }

  cv::divide(gsrc, gmask, _dst, 1, ddepth);
}
