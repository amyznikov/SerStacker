/*
 * lpg.cc
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#include "lpg.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

static void compute_gradient(const cv::Mat & src, cv::Mat & g)
{
  static const thread_local cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  constexpr int ddepth = CV_32F;

  cv::Mat gx, gy;

  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0,
      cv::BORDER_DEFAULT);

  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0,
      cv::BORDER_DEFAULT);

  cv::add(gx.mul(gx), gy.mul(gy), g);
}

// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
static void compute_laplacian(const cv::Mat & src, cv::Mat & l)
{
  static float k[5 * 5] = {
      0, 0, -1, 0, 0,
      0, -1, -2, -1, 0,
      -1, -2, 16, -2, -1,
      0, -1, -2, -1, 0,
      0, 0, -1, 0, 0,
  };

  static const thread_local cv::Mat1f K =
      cv::Mat1f(5, 5, k) / 16.;

  cv::filter2D(src, l, CV_32F, K, cv::Point(-1, -1), 0,
      cv::BORDER_REPLICATE);


  cv::multiply(l, l, l);
}

static bool downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  if( std::min(src.cols(), src.rows()) < 4 ) {
    src.copyTo(dst);
  }
  else {

    cv::pyrDown(src, dst, cv::Size(), border_mode);

    if( std::min(dst.cols, dst.rows) >= 4 ) {

      for( int l = 1; l < level; ++l ) {

        cv::pyrDown(dst, dst, cv::Size(), border_mode);

        if( std::min(dst.cols, dst.rows) < 4 ) {
          break;
        }
      }
    }
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

/**
 * lpg:
 *
 *  Create the map of
 *      alpha * laplacian ^ 2 +  beta * |gradient| ^ 2
 *
 * with
 *    alpha = k / ( k + 1)
 *    beta = 1 / (k + 1)
 *
 *
 *
 * For noise filtering purposes the input image can be optionally downscaled using cv::pyrDown(),
 * and the output map can be upscaled using cv::pyrUp().
 *
 * When average_color_channels is true the color channels of input image are averaged into
 * single channel before processing.
 *
 */
bool lpg(cv::InputArray image, cv::InputArray mask, cv::OutputArray optional_output_map,
    double k, double p, int dscale, int uscale, bool average_color_channels,
    cv::Scalar * optional_output_sharpness_metric)
{

  const cv::Mat src =
      image.getMat();

  cv::Mat s, l, g, m;

  src.convertTo(s, CV_32F, 1. / maxval(src.depth()));

  if( average_color_channels && s.channels() > 1 ) {
    reduce_color_channels(s, s, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s, s, dscale);
  }

  compute_gradient(s, g);

  if( k > 0 ) {

    compute_laplacian(s, l);

    cv::addWeighted(l, k / (k + 1.), g, 1. / (k + 1), 0, m);

    if( p > 1  ) {
      cv::pow(m, p, m);
    }

  }
  else if( p > 1 ) {

    cv::pow(g, p, m);

  }
  else {

    m = g;

  }

  if( optional_output_sharpness_metric ) {

    if( mask.empty() ) {
      *optional_output_sharpness_metric = cv::mean(m);
    }
    else {
      cv::Mat1b msk;
      cv::resize(mask, msk, m.size(), 0, 0, cv::INTER_NEAREST);
      *optional_output_sharpness_metric = cv::mean(m, msk);
    }

  }

  if( optional_output_map.needed() ) {

    if( uscale > 0 && uscale > dscale ) {
      downscale(m, m, uscale - std::max(0, dscale));
    }

    if( m.size() != src.size() ) {
      upscale(m, src.size());
    }

    optional_output_map.move(m);
  }

  return true;
}
