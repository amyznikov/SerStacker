/*
 * focus.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "focus.h"
#include "weighted_mean.h"
#include <core/debug.h>


static void compute_gradient(const cv::Mat & src, cv::Mat & dst)
{
  INSTRUMENT_REGION("");

  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  constexpr int ddepth = CV_32F;

  cv::Mat gx, gy;

  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::add(gx.mul(gx), gy.mul(gy), dst);
}

static bool downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);
  for( int l = 1; l < level; ++l ) {
    cv::pyrDown(dst, dst, cv::Size(), border_mode);
  }
  return true;
}

static bool upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> spyramid;

    spyramid.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      spyramid.emplace_back(nextSize);
    }

    for( int i = spyramid.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, spyramid[i]);
    }
  }

  return true;
}


static int get_bpp(int ddepth)
{
  switch (ddepth) {
    case CV_8U:
    case CV_8S:
      return 8;

    case CV_16U:
    case CV_16S:
      return 16;

    case CV_32S:
      return 32;
  }

  return 0;
}

static inline float square(float x)
{
  return x * x;
}

cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image, cv::OutputArray cmap,
    double eps, cv::InputArray H)
{

  INSTRUMENT_REGION("");

  cv::Mat s, ss, g, r;

  if( eps <= 0 ) {
    eps = 1e-6;
  }

  const cv::Mat src =
      image.getMat();

  if( src.depth() == CV_32F ) {
    cv::add(src, eps, s);
  }
  else {
    src.convertTo(s, CV_32F, 1. / (1 << get_bpp(src.depth())), eps);
  }

  if( true ) {
    INSTRUMENT_REGION("blur");
    downscale(s, ss, 3);
    upscale(ss, s.size());
  }
  else {
    INSTRUMENT_REGION("blur");
    static const cv::Mat1f K = cv::getGaussianKernel(15, 5, CV_32F);
    cv::sepFilter2D(s, ss, CV_32F, K, K);
  }

  compute_gradient(ss, g);

  const bool needcmap =
      cmap.needed();

  const int src_channels =
      src.channels();

  const int cn =
      (std::min)(4, src_channels);

  if( needcmap ) {
    r.create(s.size(), CV_MAKETYPE(CV_32F, cn));
  }

  cv::Scalar rr =
      cv::Scalar::all(0);

  cv::Scalar ww =
      cv::Scalar::all(0);

  for( int y = 0; y < s.rows; ++y ) {

    const float *sp = s.ptr<const float>(y);
    const float *ssp = ss.ptr<const float>(y);
    const float *gp = g.ptr<const float>(y);
    float *rp = needcmap ? r.ptr<float>(y) : nullptr;

    for( int x = 0; x < s.cols; ++x ) {
      for( int c = 0; c < cn; ++c ) {

        const float w =
            gp[x * src_channels + c];

        const float v =
            w * square(sp[x * src_channels + c] / ssp[x * src_channels + c] - 1);

        rr[c] += v;
        ww[c] += w;

        if( needcmap ) {
          rp[x * src_channels + c] = v;
        }
      }
    }
  }

  for( int c = 0; c < cn; ++c ) {
    rr[c] /= ww[c];
  }

  if( needcmap ) {
    cv::divide(r, ww, cmap);
  }

  return rr;
}


cv::Scalar c_local_contrast_measure::compute(cv::InputArray image)
{
  return compute_contrast_map(image, cv::noArray(),
      noise_eps_, cv::noArray());
}
