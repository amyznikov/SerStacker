/*
 * unsharp_mask.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "unsharp_mask.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

// totally faster for large sigma but little approximate
static void create_lpass_image(cv::InputArray src, cv::Mat & lpass, double sigma, int ddepth, double dscale)
{
  constexpr int borderType = cv::BORDER_REFLECT;

  static const auto gaussian_blur =
      [](cv::InputArray src, cv::Mat & lpass, double sigma, int ddepth, double dscale) {
        const cv::Mat1f G = cv::getGaussianKernel(2 * (std::max)(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
        if ( std::abs(dscale-1) > std::numeric_limits<float>::epsilon() ) {
          G *= std::sqrt(dscale);
        }
        cv::sepFilter2D(src, lpass, ddepth, G, G, cv::Point(-1, -1), 0, borderType);
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
    gaussian_blur(src, lpass, sigma, ddepth, dscale);
  }
  else {

    std::vector<cv::Size> size_history;

    const double delta = sqrt(sigma * sigma - 2 * Ci) / (1 << pyramid_level)  ;

    size_history.emplace_back(src.size());
    cv::pyrDown(src, lpass, cv::Size(), borderType);

    for ( int j = 1; j < pyramid_level; ++j ) {
      size_history.emplace_back(lpass.size());
      cv::pyrDown(lpass, lpass, cv::Size(), borderType);
    }

    if ( delta > 0 ) {
      gaussian_blur(lpass, lpass, delta, ddepth, dscale);
    }
    else {
      lpass.convertTo(lpass, ddepth, dscale);
    }

    for( int j = size_history.size() - 1; j >= 0; --j ) {
      cv::pyrUp(lpass, lpass, size_history[j]);
    }
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
    cv::Mat lpass;
    create_lpass_image(src, lpass, sigma, src.depth(), 1);
    cv::addWeighted(src, 1. / (1. - alpha), lpass, -alpha / (1. - alpha), 0, dst);
  }

  if ( outmax > outmin ) {
    cv::Mat & dstm = dst.getMatRef();
    cv::min(dstm, outmax, dstm);
    cv::max(dstm, outmin, dstm);
  }
}

template<class _Tp1, class _Tp2>
static bool _unsharp_mask_with_mask(cv::InputArray _src, cv::InputArray _srcmask, cv::OutputArray _dst,
    double sigma, double w, double outmin, double outmax)
{
  const cv::Mat src = _src.getMat();

  cv::Mat bmask, fmask;
  cv::Mat lpass;

  const int mask_depth = _srcmask.depth();
  const int mask_cn = _srcmask.channels();

  if ( mask_cn == 1 ) {
    if ( mask_depth == CV_8U ) {
      bmask = _srcmask.getMat();
    }
    else {
      cv::compare(_srcmask, 0, bmask, cv::CMP_GT);
    }
  }
  else {
    reduce_color_channels(_srcmask, bmask, cv::REDUCE_MIN);
    if ( mask_depth != CV_8U ) {
      cv::compare(bmask, 0, bmask, cv::CMP_GT);
    }
  }

  src.copyTo(lpass, bmask);
  create_lpass_image(lpass, lpass, sigma, CV_32F, 1);
  create_lpass_image(bmask, fmask, sigma, CV_32F, 1. / 255);

  _dst.create(src.size(), CV_MAKETYPE(cv::DataType<_Tp2>::depth, src.channels()));
  cv::Mat & dst = _dst.getMatRef();

  parallel_for(0, lpass.rows, [&bmask, &fmask, &lpass, &src, &dst, w, outmin, outmax](const auto & range) {
    const double alpha = 1. / (1. - w);
    const double beta = -w / (1. - w);
    const int cn = src.channels();

    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const float * lpassp = lpass.ptr<float>(y);
      const float * fmaskp = fmask.ptr<const float>(y);
      const uint8_t * smaskp = bmask.ptr<const uint8_t>(y);

      const _Tp1 * srcp = src.ptr<const _Tp1>(y);
      _Tp2 * __restrict dstp = dst.ptr<_Tp2>(y);

      if ( !(outmax > outmin) ) {
        // little faster path
        for ( int x = 0, nx = lpass.cols; x < nx; ++x, lpassp += cn, srcp += cn, dstp += cn ) {
          if ( smaskp[x] ) {
            for ( int c = 0; c < cn; ++c ) {
              const double v = alpha * srcp[c] + beta * (lpassp[c] / fmaskp[x]);
              dstp[c] = cv::saturate_cast<_Tp2>(v);
            }
          }
        }
      }
      else {
        // little slower path with range checks
        for ( int x = 0, nx = lpass.cols; x < nx; ++x, lpassp += cn, srcp += cn, dstp += cn ) {
          if ( smaskp[x] ) {
            for ( int c = 0; c < cn; ++c ) {
              const double v = std::clamp(alpha * srcp[c] + beta * (lpassp[c] / fmaskp[x]), outmin, outmax);
              dstp[c] = cv::saturate_cast<_Tp2>(v);
            }
          }
        }
      }
    }

  });

  return true;
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
    CV_DISPATCH2(src.depth(), ddepth, _unsharp_mask_with_mask, src, srcmask, dst, sigma, alpha, outmin, outmax);
    CF_ERROR("Unsupported pixel depth encountered: ddepth=%d", ddepth);
    return false;
  }

  return true;
}


double hpass_norm(cv::InputArray src, double sigma, cv::InputArray _mask,
    enum cv::NormTypes normType)
{
  static const thread_local cv::Mat1f G =
      cv::getGaussianKernel(5, sigma, CV_32F);

  cv::Mat lpass;
  cv::Mat mask;

  if( !_mask.empty() ) {
    if( _mask.channels() == 1 ) {
      if( _mask.depth() == CV_8U ) {
        mask = _mask.getMat();
      }
      else {
        cv::compare(_mask, 0, mask, cv::CMP_GT);
      }
    }
    else {
      reduce_color_channels(_mask, mask, cv::REDUCE_MIN);
      if( mask.depth() != CV_8U ) {
        cv::compare(mask, 0, mask, cv::CMP_GT);
      }
    }
  }

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

  return v;
}

