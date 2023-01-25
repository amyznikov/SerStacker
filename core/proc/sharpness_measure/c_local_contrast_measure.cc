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


static void compute_gradient(const cv::Mat & src, cv::Mat & g, double delta = 0)
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

  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), delta, cv::BORDER_DEFAULT);
  cv::magnitude(gx, gy, g);
}


// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
static void compute_laplacian(const cv::Mat & src, cv::Mat & l, double delta)
{
  static float k[5 * 5] = {
      0, 0, -1, 0, 0,
      0, -1, -2, -1, 0,
      -1, -2, 16, -2, -1,
      0, -1, -2, -1, 0,
      0, 0, -1, 0, 0,
  };

  static const cv::Mat1f K =
      cv::Mat1f(5, 5, k) / 16.;

  cv::filter2D(src, l, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  //cv::multiply(l, l, l);
  cv::absdiff(l, 0, l);
  if( delta > 0 ) {
    cv::add(l, delta, l);
  }
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

cv::Scalar c_local_contrast_measure::compute_contrast_map(cv::InputArray image,
    cv::OutputArray output_contrast_map, double eps, int dscale, bool avgchannel)
{

  INSTRUMENT_REGION("");

  const cv::Mat src =
      image.getMat();

  cv::Mat s1, s2, e, d, mg, gg, map;

  src.convertTo(s1, CV_32F, 1. / maxval(src.depth()));

  if( avgchannel && s1.channels() > 1 ) {
    reduce_color_channels(s1, s1, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s1, s1, dscale);
  }

  static const cv::Mat1b SE(3, 3, 255);

  cv::morphologyEx(s1, mg, cv::MORPH_GRADIENT, SE);

  downscale(s1, s2, 2);
  upscale(s2, s1.size());
  compute_gradient(s2, gg);

  cv::divide(mg, s2 + cv::Scalar::all(eps), map);

  cv::Scalar rv = weighted_mean(map, gg);

  if( output_contrast_map.needed() ) {
    cv::multiply(map, gg, output_contrast_map);
    cv::divide(output_contrast_map, cv::sum(gg), output_contrast_map);
  }

  return rv;
}

void c_local_contrast_measure::set_dscale(int v)
{
  dscale_ = v;
}

int c_local_contrast_measure::dscale() const
{
  return dscale_;
}

void c_local_contrast_measure::set_eps(double v)
{
  eps_ = v;
}

double c_local_contrast_measure::eps() const
{
  return eps_;
}

void c_local_contrast_measure::set_avgchannel(bool v)
{
  avgchannel_ = v;
}

bool c_local_contrast_measure::avgchannel() const
{
  return avgchannel_;
}

cv::Scalar c_local_contrast_measure::compute(cv::InputArray image) const
{
  return compute_contrast_map(image, cv::noArray(),
      eps_, dscale_, avgchannel_);
}

cv::Scalar c_local_contrast_measure::create_sharpeness_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return compute_contrast_map(image, output_map,
      eps_, dscale_, avgchannel_);
}

#if 0
static void compute_fft(const cv::Mat & src, cv::Mat & dst)
{

  static const auto prepare_channel =
      [](cv::Mat & img) {

        cv::Mat1f image = img;

        for ( int y = 0; y < image.rows; ++y ) {
          for ( int x = 0; x < image.cols; ++x ) {
            if( (x + y) & 0x1 ) {
              image[y][x] = -image[y][x];
            }
          }
        }
      };

  std::vector<cv::Mat> channels;

  const int cn =
      src.channels();

  if ( cn == 1 ) {
    channels.emplace_back(src);
  }
  else {
    cv::split(src, channels);
  }

  //cv::Scalar ww = cv::Scalar::all(0);
  dst.create(src.size(), CV_MAKETYPE(CV_32F, cn));


  for ( int c = 0; c < cn; ++c ) {

    prepare_channel(channels[c]); // instead of fftSwapQuadrants(channels[c]);

    cv::dft(channels[c], channels[c],
        cv::DFT_COMPLEX_OUTPUT);

    cv::Mat2f s =
        channels[c];

    for( int y = 0; y < s.rows; ++y ) {

      float *dstp = dst.ptr<float>(y) + c;

      for( int x = 0; x < s.cols; ++x ) {

        const float re = s[y][x][0];
        const float im = s[y][x][1];
        const float w = sqrt(re * re + im * im);
        //ww[c] += w;
        dstp[x * cn] = w;
      }
    }
  }

  //cv::divide(dst, ww, dst);
}
#endif
