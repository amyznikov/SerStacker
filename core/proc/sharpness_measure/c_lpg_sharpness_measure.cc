/*
 * c_lpg_sharpness_measure.cc
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#include "c_lpg_sharpness_measure.h"
#include <core/proc/lpg.h>

//#include <core/proc/reduce_channels.h>
#include <core/debug.h>

//
//static void compute_gradient(const cv::Mat & src, cv::Mat & g)
//{
//  INSTRUMENT_REGION("");
//
//  static thread_local const cv::Matx<float, 1, 5> K(
//      (+1.f / 12),
//      (-8.f / 12),
//      0.f,
//      (+8.f / 12),
//      (-1.f / 12));
//
//  constexpr int ddepth = CV_32F;
//
//  cv::Mat gx, gy;
//
//  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
//  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
//  cv::add(gx.mul(gx), gy.mul(gy), g);
//}
//
//
//// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
//static void compute_laplacian(const cv::Mat & src, cv::Mat & l)
//{
//  INSTRUMENT_REGION("");
//
//  static float k[5 * 5] = {
//      0, 0, -1, 0, 0,
//      0, -1, -2, -1, 0,
//      -1, -2, 16, -2, -1,
//      0, -1, -2, -1, 0,
//      0, 0, -1, 0, 0,
//  };
//
//  static const cv::Mat1f K =
//      cv::Mat1f(5, 5, k) / 16.;
//
//  cv::filter2D(src, l, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  cv::multiply(l, l, l);
//}
//
//
//static bool downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
//{
//  if( std::min(src.cols(), src.rows()) < 4 ) {
//    src.copyTo(dst);
//  }
//  else {
//
//    cv::pyrDown(src, dst, cv::Size(), border_mode);
//
//    if( std::min(dst.cols, dst.rows) >= 4 ) {
//
//      for( int l = 1; l < level; ++l ) {
//
//        cv::pyrDown(dst, dst, cv::Size(), border_mode);
//
//        if( std::min(dst.cols, dst.rows) < 4 ) {
//          break;
//        }
//      }
//    }
//  }
//
//  return true;
//}
//
//static bool upscale(cv::Mat & image, cv::Size dstSize)
//{
//  const cv::Size inputSize = image.size();
//
//  if( inputSize != dstSize ) {
//
//    std::vector<cv::Size> spyramid;
//
//    spyramid.emplace_back(dstSize);
//
//    while (42) {
//      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
//      if( nextSize == inputSize ) {
//        break;
//      }
//      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
//        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
//            nextSize.width, nextSize.height,
//            inputSize.width, inputSize.height);
//        return false;
//      }
//      spyramid.emplace_back(nextSize);
//    }
//
//    for( int i = spyramid.size() - 1; i >= 0; --i ) {
//      cv::pyrUp(image, image, spyramid[i]);
//    }
//  }
//
//  return true;
//}
//
//static double maxval(int ddepth)
//{
//  switch (CV_MAT_DEPTH(ddepth)) {
//    case CV_8U:
//      return UINT8_MAX;
//    case CV_8S:
//      return INT8_MAX;
//    case CV_16U:
//      return UINT16_MAX;
//    case CV_16S:
//      return INT16_MAX;
//    case CV_32S:
//      return INT32_MAX;
//  }
//
//  return 1;
//}

c_lpg_sharpness_measure::c_lpg_sharpness_measure()
{

}

c_lpg_sharpness_measure::c_lpg_sharpness_measure(const c_lpg_options & opts) :
    options_(opts)
{
}

void c_lpg_sharpness_measure::set_k(double v)
{
  options_.k = v;
}

double c_lpg_sharpness_measure::k() const
{
  return options_.k;
}

void c_lpg_sharpness_measure::set_dscale(int v)
{
  options_.dscale = v;
}

int c_lpg_sharpness_measure::dscale() const
{
  return options_.dscale;
}

void c_lpg_sharpness_measure::set_uscale(int v)
{
  options_.uscale = v;
}

int c_lpg_sharpness_measure::uscale() const
{
  return options_.uscale;
}

void c_lpg_sharpness_measure::set_squared(bool v)
{
  options_.squared = v;
}

bool c_lpg_sharpness_measure::squared() const
{
  return options_.squared;
}

void c_lpg_sharpness_measure::set_avgchannel(bool v)
{
  options_.avgchannel = v;
}

bool c_lpg_sharpness_measure::avgchannel() const
{
  return options_.avgchannel;
}

cv::Scalar c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Scalar rv;
  compute(image, mask,
      cv::noArray(),
      options_.k,
      options_.dscale,
      options_.uscale,
      options_.squared,
      options_.avgchannel,
      &rv);
  return rv;
}

bool c_lpg_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return compute(image, cv::noArray(),
      output_map,
      options_.k,
      options_.dscale,
      options_.uscale,
      options_.squared,
      options_.avgchannel,
      nullptr);
}

bool c_lpg_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map,
    const c_lpg_options & opts)
{
  return compute(image, cv::noArray(),
      output_map,
      opts.k,
      opts.dscale,
      opts.uscale,
      opts.squared,
      opts.avgchannel,
      nullptr);
}

bool c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
    double k, int dscale, int uscale, bool squared, bool avgchannel,
    cv::Scalar * output_sharpness_metric)
{
  return lpg(image, mask, output_map,
    k, dscale, uscale, squared, avgchannel,
    output_sharpness_metric);
}

bool c_lpg_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
    const c_lpg_options & opts,
    cv::Scalar * output_sharpness_metric)
{
  return lpg(image, mask, output_map,
      opts.k, opts.dscale, opts.uscale, opts.squared, opts.avgchannel,
      output_sharpness_metric);
}
