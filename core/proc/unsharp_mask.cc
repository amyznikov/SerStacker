/*
 * unsharp_mask.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "unsharp_mask.h"
#if HAVE_TBB
# include <tbb/tbb.h>
#endif
#include <core/debug.h>

// totally faster for large sigma but little approximate
static void create_lpass_image(cv::InputArray src, cv::Mat & lpass, double sigma)
{
  constexpr int borderType = cv::BORDER_REFLECT;

  static const auto gaussian_blur =
      [](cv::InputArray src, cv::Mat & lpass, double sigma) {
        const cv::Mat1f G = cv::getGaussianKernel(2 * (std::max)(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
        cv::sepFilter2D(src, lpass, -1, G,  G, cv::Point(-1, -1), 0, borderType);
      };

  int pyramid_level = 0;
  int Ci = 0;

  if( sigma > 2 ) {

    int min_size = (std::min)(src.rows(), src.cols());
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

  const double delta = sqrt(sigma * sigma - 2 * Ci) / (1 << pyramid_level)  ;

  size_history.emplace_back(src.size());
  cv::pyrDown(src, lpass, cv::Size(), borderType);

  for ( int j = 1; j < pyramid_level; ++j ) {
    size_history.emplace_back(lpass.size());
    cv::pyrDown(lpass, lpass, cv::Size(), borderType);
  }

  if ( delta > 0 ) {
    gaussian_blur(lpass, lpass, delta);
  }

  for( int j = size_history.size() - 1; j >= 0; --j ) {
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
    create_lpass_image(src, lpass, sigma);
    cv::addWeighted(src, 1. / (1. - alpha), lpass, -alpha / (1. - alpha), 0, dst);
  }

  if ( outmax > outmin ) {
    cv::min(dst.getMat(), outmax, dst.getMatRef());
    cv::max(dst.getMat(), outmin, dst.getMatRef());
  }
}




template<class TSRC, class TDST>
static void _unsharp_mask_with_mask(cv::InputArray _src, cv::InputArray srcmask, cv::OutputArray _dst,
    double sigma, double w, double outmin, double outmax)
{
#if HAVE_TBB
  typedef tbb::blocked_range<int> tbb_range;
#endif

  const cv::Mat src = _src.getMat();
  const cv::Mat1b mask = srcmask.getMat();

  cv::Mat lpass, fmask;

  mask.convertTo(fmask, CV_32F, 1. / 255);
  src.convertTo(lpass, CV_32F);
  lpass.setTo(0, ~mask);

  create_lpass_image(lpass, lpass, sigma);
  create_lpass_image(fmask, fmask, sigma);

  _dst.create(src.size(), CV_MAKETYPE(cv::DataType<TDST>::depth, src.channels()));
  cv::Mat & dst = _dst.getMatRef();

#if HAVE_TBB
  tbb::parallel_for(tbb_range(0, lpass.rows, 128),
      [&mask, &fmask, &lpass, &src, &dst, w, outmin, outmax](const tbb_range & range) {
#endif
        const double alpha = 1. / (1. - w);
        const double beta = -w / (1. - w);
        const int cn = src.channels();

#if HAVE_TBB
        for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {
#else
        for ( int y = 0, ny = lpass.rows; y < ny; ++y ) {
#endif
          const float * lpassp = lpass.ptr<float>(y);
          const float * fmaskp = fmask.ptr<const float>(y);
          const uint8_t * smaskp = mask[y];

          const TSRC * srcp = src.ptr<const TSRC>(y);
          TDST * dstp = dst.ptr<TDST>(y);

          for ( int x = 0, nx = lpass.cols; x < nx; ++x, lpassp += cn, srcp += cn, dstp += cn ) {
            if ( smaskp[x] ) {
              for ( int c = 0; c < cn; ++c ) {

                double v = alpha * srcp[c] + beta * (lpassp[c] / fmaskp[x]);
                if ( outmax > outmin ) {
                  if ( v < outmin ) {
                    v = outmin;
                  }
                  else if ( v > outmax ) {
                    v = outmax;
                  }
                }

                dstp[c] = (TDST)(v);
              }
            }
          }
        }
#if HAVE_TBB
      });
#endif
}

bool unsharp_mask(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst,
    double sigma,
    double alpha,
    double outmin,
    double outmax)
{

  if ( srcmask.empty() ) {
    unsharp_mask(src, dst, sigma, alpha, outmin, outmax);
    return true;
  }

  if ( sigma <= 0 || alpha <= 0 ) {
    src.copyTo(dst);
  }
  else {

    const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

    switch ( ddepth ) {
    case CV_8U :
      if ( outmax <= outmin ) {
        outmin = 0, outmax = UINT8_MAX;
      }
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, uint8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;

    case CV_8S :
      if ( outmax <= outmin ) {
        outmin = -INT8_MIN, outmax = INT8_MAX;
      }
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, int8_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    case CV_16U :
      if ( outmax <= outmin ) {
        outmin = 0, outmax = UINT16_MAX;
      }
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, uint16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    case CV_16S :
      if ( outmax <= outmin ) {
        outmin = INT16_MIN, outmax = INT16_MAX;
      }
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, int16_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    case CV_32S :
      if ( outmax <= outmin ) {
        outmin = INT32_MIN, outmax = INT32_MAX;
      }
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, int32_t>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    case CV_32F :
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, float>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    case CV_64F :
      switch ( src.depth() ) {
      case CV_8U :
        _unsharp_mask_with_mask<uint8_t, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_8S :
        _unsharp_mask_with_mask<int8_t, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16U :
        _unsharp_mask_with_mask<uint16_t, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_16S :
        _unsharp_mask_with_mask<int16_t, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32S :
        _unsharp_mask_with_mask<int32_t, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_32F :
        _unsharp_mask_with_mask<float, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      case CV_64F :
        _unsharp_mask_with_mask<double, double>(src, srcmask, dst, sigma, alpha, outmin, outmax);
        break;
      }
      break;
    default : /*cv::minMaxLoc(src, &outmin, &outmax); */
      CF_ERROR("Unsupported pixel depth encountered: ddepth=%d", ddepth);
      return false;
    }
  }

  return true;
}


double hpass_norm(cv::InputArray src, double sigma, cv::InputArray mask,
    enum cv::NormTypes normType)
{
  static const thread_local cv::Mat1f G =
      cv::getGaussianKernel(5, sigma, CV_32F);

  cv::Mat lpass;

  cv::sepFilter2D(src, lpass,
      src.depth(),
      G, G,
      cv::Point(-1, -1),
      0,
      cv::BORDER_REFLECT101);

  double v =
      cv::norm(src, lpass, normType,
          mask);

  switch (normType) {
    case cv::NORM_L1:
    case cv::NORM_L2SQR:
      if( mask.size() == src.size() ) {
        v /= cv::countNonZero(mask);
      }
      else {
        v /= src.size().area();
      }
      break;
    case cv::NORM_L2:
      if( mask.size() == src.size() ) {
        v /= sqrt(cv::countNonZero(mask));
      }
      else {
        v /= sqrt(src.size().area());
      }
      break;
    default:
      break;
  }

//  if( mask.size() == src.size() ) {
//    v /= cv::countNonZero(mask);
//  }
//  else {
//    v /= src.size().area();
//  }

  return v;
}
