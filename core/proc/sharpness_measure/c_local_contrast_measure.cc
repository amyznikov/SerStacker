/*
 * c_local_contrast_measure.cc
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#include "c_local_contrast_measure.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/weighted_mean.h>
#include <core/debug.h>


//static void compute_gradient(const cv::Mat & src, cv::Mat & g, double delta = 0)
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
//  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
//  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
//  cv::magnitude(gx, gy, g);
//}

//
//// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
//static void compute_laplacian(const cv::Mat & src, cv::Mat & l, double delta)
//{
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
//  //cv::multiply(l, l, l);
//  cv::absdiff(l, 0, l);
//  if( delta > 0 ) {
//    cv::add(l, delta, l);
//  }
//}



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

static double maxval(int ddepth)
{
  switch (CV_MAT_DEPTH(ddepth)) {
    case CV_8U:
      return UINT8_MAX;
    case CV_8S:
      return INT8_MAX;
    case CV_16U:
      return UINT16_MAX;
    case CV_16S:
      return INT16_MAX;
    case CV_32S:
      return INT32_MAX;
  }

  return 1;
}

bool c_local_contrast_measure::compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_contrast_map,
    double eps, int dscale, bool avgchannel,
    cv::Scalar * output_sharpness_measure)
{
  INSTRUMENT_REGION("");

  const cv::Mat src =
      image.getMat();

  const cv::Size src_size =
      src.size();

  static const cv::Mat1b SE(5, 5, 255);

  cv::Mat s1, s2, e, d, mg, gg, map;

  src.convertTo(s1, CV_32F, 1. / maxval(src.depth()));

  if( /*avgchannel && */s1.channels() > 1 ) {
    reduce_color_channels(s1, s1, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s1, s1, dscale);
  }

  cv::morphologyEx(s1, mg, cv::MORPH_GRADIENT, SE);

  downscale(s1, s1, 2);
  upscale(s1, mg.size());

  cv::multiply(mg, s1, mg, 1.0 / cv::sum(s1)[0]);

  if( output_sharpness_measure ) {
    *output_sharpness_measure =
        cv::sum(mg);
  }

  if( output_contrast_map.needed() ) {

    output_contrast_map.move(mg);
    if( dscale > 0 ) {
      upscale(output_contrast_map.getMatRef(), src_size);
    }

  }



//  if( output_contrast_map.needed() ) {
//    output_contrast_map.move(mg);
//    if( dscale > 0 ) {
//      upscale(output_contrast_map.getMatRef(), src_size);
//    }
//  }


//  downscale(s1, s2, 2);
//  upscale(s2, s1.size());
//  compute_gradient(s2, gg);
//
//  cv::divide(mg, s2 + cv::Scalar::all(eps), map);
//
//  if( output_sharpness_measure ) {
//    * output_sharpness_measure =
//        weighted_mean(map, gg);
//  }
//
//  if( output_contrast_map.needed() ) {
//
//    cv::multiply(map, gg, output_contrast_map);
//
//    cv::divide(output_contrast_map, cv::sum(gg),
//        output_contrast_map);
//  }

  return true;
}

void c_local_contrast_measure::set_dscale(int v)
{
  _opts.dscale = v;
}

int c_local_contrast_measure::dscale() const
{
  return _opts.dscale;
}

void c_local_contrast_measure::set_eps(double v)
{
  _opts.eps = v;
}

double c_local_contrast_measure::eps() const
{
  return _opts.eps;
}

void c_local_contrast_measure::set_avgchannel(bool v)
{
  _opts.avgchannel = v;
}

bool c_local_contrast_measure::avgchannel() const
{
  return _opts.avgchannel;
}

cv::Scalar c_local_contrast_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Scalar rv;

  compute(image, mask,
      cv::noArray(),
      _opts.eps,
      _opts.dscale,
      _opts.avgchannel,
      &rv);

  return rv;
}

bool c_local_contrast_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return compute(image, cv::noArray(), output_map,
      _opts.eps, _opts.dscale, _opts.avgchannel);
}
