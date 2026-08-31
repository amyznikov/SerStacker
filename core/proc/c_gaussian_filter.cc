/*
 * c_gaussian_filter.cc
 *
 *  Created on: Aug 18, 2021
 *      Author: amyznikov
 */

#include "c_gaussian_filter.h"
#include <core/proc/divide.h>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

c_gaussian_filter::c_gaussian_filter()
{
}

c_gaussian_filter::c_gaussian_filter(double sigmaX, double sigmaY, const cv::Size & ksize, double scale) :
  _sigmaX(sigmaX), _sigmaY(sigmaY), _ksize(ksize), _scale(scale)
{
  create_gaussian_kernels(_Kx, _Ky, CV_32F, _ksize, _sigmaX, _sigmaY);
}

void c_gaussian_filter::create_gaussian_kernels(cv::Mat & kx, cv::Mat & ky, int ktype, cv::Size & ksize, double sigmax, double sigmay)
{
  // Auto setup kernel size from sigma if not specified explicitly

  const int kdepth = ktype < 0 ? CV_32F : CV_MAT_DEPTH(ktype);

  if( sigmax < 0 ) {
    sigmax = sigmay > 0 ? sigmay : 0;
  }
  if( sigmay < 0 ) {
    sigmay = sigmax > 0 ? sigmax : 0;
  }

  if ( ksize.width <= 0 ) {
    if ( sigmax > 0 ) {
      ksize.width = std::max(3, 2 * cvRound(sigmax * (kdepth == CV_8U ? 3 : 4)) + 1);
    }
    else {
      ksize.width = 1;
    }
  }

  if ( ksize.height <= 0 ) {
    if ( sigmay > 0  ) {
      ksize.height = std::max(3, 2 * cvRound(sigmay * (kdepth == CV_8U ? 3 : 4)) + 1);
    }
    else {
      ksize.height = 1;
    }
  }

  CV_Assert(ksize.width > 0 && ksize.width % 2 == 1 && ksize.height > 0 && ksize.height % 2 == 1);

  if( ksize.width > 1 ) {
    kx = cv::getGaussianKernel(ksize.width, sigmax,
        std::max(kdepth, CV_32F));
  }
  else {
    kx = cv::Mat1f::ones(1, 1);
  }

  if ( ksize.height > 1 ) {
    ky = cv::getGaussianKernel(ksize.height, sigmay,
        std::max(kdepth, CV_32F));
  }
  else {
    ky = cv::Mat1f::ones(1, 1);
  }
}

void c_gaussian_filter::apply(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst, int borderType, int ddepth) const
{
  const cv::Size src_size = _src.size();
  const int src_type = _src.type();

  if ( _dst.fixedType() ) {
    ddepth = _dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = _src.depth();
  }

  if ( _Kx.rows == 1 &&  _Ky.rows == 1 ) {
    _src.getMat().convertTo(_dst, ddepth, _scale);
    return;
  }

  if ( _mask.empty() ) {
    cv::sepFilter2D(_src, _dst,
        ddepth,
        _scale * _Kx, _Ky,
        cv::Point(-1, -1),
        0,
        borderType);
    return;
  }

  cv::Mat src, gsrc;
  cv::Mat mask, gmask;

  if ( _mask.channels() == 1 ) {
    if ( _mask.depth() == CV_8U ) {
      mask = _mask.getMat();
    }
    else {
      cv::compare(_mask, 0, mask, cv::CMP_GT);
    }
  }
  else {
    reduce_color_channels(_mask, mask, cv::REDUCE_MIN);
    if ( mask.depth() != CV_8U ) {
      cv::compare(mask, 0, mask, cv::CMP_GT);
    }
  }

  _src.getMat().copyTo(src, mask);

  cv::sepFilter2D(src, gsrc,
      CV_32F,
      _scale * _Kx, _Ky,
      cv::Point(-1, -1),
      0,
      borderType);

  cv::sepFilter2D(mask, gmask,
      CV_32F,
      (1.0 / 255.0) * _Kx, _Ky,
      cv::Point(-1, -1),
      0,
      borderType);

    divideImages(gsrc, gmask, _dst, 1e-5, ddepth);
}

void gaussian_filter(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    const cv::Size2f & sigma, const cv::Size & _ksize,
    double scale, double delta,
    cv::BorderTypes borderType,
    const cv::Scalar & borderValue)
{
  cv::Mat src, mask, blured;

  const c_gaussian_filter G(sigma.width, sigma.height, _ksize, scale);
  const cv::Size ksize = G.ksize();
  const cv::Size srcSize = _src.size();

  const int ddepth = _dst.fixedType() ? _dst.depth() : std::max(_src.depth(), CV_32F);

  int borderx = 0, bordery = 0;

  if( borderType != cv::BORDER_WRAP && borderType != cv::BORDER_TRANSPARENT ) {
    src = _src.getMat();
    mask = _mask.getMat();
  }
  else {
    borderx = ksize.width > 1 ? ksize.width / 2 : 0;
    bordery = ksize.height > 1 ? ksize.height / 2 : 0;
    cv::copyMakeBorder(_src, src, bordery, bordery, borderx, borderx, borderType, borderValue);
    if( !_mask.empty() ) {
      cv::copyMakeBorder(_mask, mask, bordery, bordery, borderx, borderx, cv::BORDER_REPLICATE);
    }
    borderType = cv::BORDER_DEFAULT;
  }


  G.apply(src, mask, blured, borderType, ddepth);

  if( borderx >= 1 || bordery >= 1 ) {
    blured = blured(cv::Rect(borderx, bordery, srcSize.width, srcSize.height));
  }

  if( delta != 0 ) {
    cv::add(blured, cv::Scalar::all(delta), _dst, cv::noArray(), ddepth);
  }
  else if( ddepth != blured.depth() ) {
    blured.convertTo(_dst, ddepth);
  }
  else if( borderx >= 1 || bordery >= 1 ) {
    blured.copyTo(_dst);
  }
  else {
    _dst.assign(blured);
  }
}

void gaussian_hpass_filter(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    const cv::Size2f & sigma, const cv::Size & _ksize,
    double scale, double delta,
    cv::BorderTypes borderType,
    const cv::Scalar & borderValue)
{
  cv::Mat src, mask, blured, filtered;

  const c_gaussian_filter G(sigma.width, sigma.height, _ksize, scale);
  const cv::Size ksize = G.ksize();
  const cv::Size srcSize = _src.size();

  const int ddepth = _dst.fixedType() ? _dst.depth() : std::max(_src.depth(), CV_32F);

  int borderx = 0, bordery = 0;

  if( borderType != cv::BORDER_WRAP ) {
    src = _src.getMat();
    mask = _mask.getMat();
  }
  else {
    borderx = ksize.width > 1 ? ksize.width / 2 : 0;
    bordery = ksize.height > 1 ? ksize.height / 2 : 0;
    cv::copyMakeBorder(_src, src, bordery, bordery, borderx, borderx, borderType, borderValue);
    if( !_mask.empty() ) {
      cv::copyMakeBorder(_mask, mask, bordery, bordery, borderx, borderx, cv::BORDER_REPLICATE);
    }
    borderType = cv::BORDER_DEFAULT;
  }

  G.apply(src, mask, blured, borderType, ddepth);
  cv::subtract(src, blured, filtered);

  if( delta != 0 ) {
    cv::add(filtered, cv::Scalar::all(delta), filtered);
  }

  if( borderx < 1 && bordery < 1 ) {
    _dst.assign(filtered);
  }
  else {
    filtered(cv::Rect(borderx, bordery, srcSize.width, srcSize.height)).copyTo(_dst);
  }
}

//void gaussian_hpass_filter(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
//    const cv::Size2f & sigma, const cv::Size & _ksize,
//    double scale, double delta,
//    cv::BorderTypes borderType,
//    const cv::Scalar & borderValue)
//{
//  cv::Mat src, mask, blured, filtered;
//
//  const c_gaussian_filter G(sigma.width, sigma.height, _ksize, scale);
//  const cv::Size ksize = G.ksize();
//  const cv::Size srcSize = _src.size();
//  const int ddepth = _dst.fixedType() ? _dst.depth() : std::max(_src.depth(), CV_32F);
//
//  int borderx = 0, bordery = 0;
//
//  if( borderType != cv::BORDER_WRAP ) {
//    if( _mask.empty() ) {
//      src = _src.getMat();
//    }
//    else {
//      mask = _mask.getMat();
//      _src.getMat().copyTo(src, mask);
//    }
//  }
//  else {
//    borderx = ksize.width > 1 ? ksize.width / 2 : 0;
//    bordery = ksize.height > 1 ? ksize.height / 2 : 0;
//    cv::copyMakeBorder(_src, src, bordery, bordery, borderx, borderx, borderType, borderValue);
//    if( !_mask.empty() ) {
//      cv::copyMakeBorder(_mask, mask, bordery, bordery, borderx, borderx, cv::BORDER_REPLICATE);
//      src.setTo(0, ~mask);
//    }
//    borderType = cv::BORDER_DEFAULT;
//  }
//
//  G.apply(src, mask, blured, borderType, ddepth);
//  cv::subtract(src, blured, filtered);
//  if( delta != 0 ) {
//    cv::add(filtered, cv::Scalar::all(delta), filtered);
//  }
//
//  if( borderx < 1 && bordery < 1 ) {
//    _dst.assign(filtered);
//  }
//  else {
//    filtered(cv::Rect(borderx, bordery, srcSize.width, srcSize.height)).copyTo(_dst);
//  }
//}
