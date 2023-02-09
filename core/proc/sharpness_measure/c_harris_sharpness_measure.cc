/*
 * c_harris_sharpness_measure.cc
 *
 *  Created on: Jan 25, 2023
 *      Author: amyznikov
 */

#include "c_harris_sharpness_measure.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>



static void compute_harris_map(const cv::Mat & src, double k, cv::Mat & M)
{
  INSTRUMENT_REGION("");

  static thread_local const cv::Matx<float, 1, 5> Kx(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  static thread_local const cv::Matx<float, 5, 1> Ky = Kx.t();


  constexpr int ddepth = CV_32F;

  cv::Mat gx, gy, gxx, gyy, gxy, D, T;
  cv::filter2D(src, gx, ddepth, Kx, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(src, gy, ddepth, Ky, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  cv::filter2D(gx, gxx, ddepth, Kx, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(gy, gyy, ddepth, Ky, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  cv::filter2D(gx, gxy, ddepth, Ky, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  // Eigenvectors and eigenvalues of real symmetric matrices.pdf
  // A = | a b |
  //     | b d |
  //

  const cv::Mat & a = gxx;
  const cv::Mat & b = gxy;
  const cv::Mat & d = gyy;

  // Determinant D:
  cv::absdiff(a.mul(d), b.mul(b), D);

  // Trace T:
  cv::add(a, d, T); //  T = a + d;

  // Metric: D + k * T * T
  cv::scaleAdd(T.mul(T), k, D, M);
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



void c_harris_sharpness_measure::set_k(double v)
{
  k_ = v;
}

double c_harris_sharpness_measure::k() const
{
  return k_;
}

void c_harris_sharpness_measure::set_dscale(int v)
{
  dscale_ = v;
}

int c_harris_sharpness_measure::dscale() const
{
  return dscale_;
}

void c_harris_sharpness_measure::set_uscale(int v)
{
  uscale_ = v;
}

int c_harris_sharpness_measure::uscale() const
{
  return uscale_;
}

void c_harris_sharpness_measure::set_avgchannel(bool v)
{
  avgchannel_ = v;
}

bool c_harris_sharpness_measure::avgchannel() const
{
  return avgchannel_;
}

cv::Scalar c_harris_sharpness_measure::compute(cv::InputArray image) const
{
  cv::Scalar rv;
  compute(image, cv::noArray(), k_, dscale_, uscale_, avgchannel_, &rv);
  return rv;
}

bool c_harris_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  return compute(image, output_map, k_, dscale_, uscale_, avgchannel_);
}

bool c_harris_sharpness_measure::compute(cv::InputArray image, cv::OutputArray output_map,
    double k, int dscale, int uscale, bool avgchannel,
    cv::Scalar * output_sharpness_measure)
{
  INSTRUMENT_REGION("");

  const cv::Mat src =
      image.getMat();

  cv::Mat s, l, g, m;

  src.convertTo(s, CV_32F, 1. / maxval(src.depth()));

  if( avgchannel && s.channels() > 1 ) {
    reduce_color_channels(s, s, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s, s, dscale);
  }

  compute_harris_map(s, k, m);

  if( output_sharpness_measure ) {
    *output_sharpness_measure =
        m.empty() ? cv::Scalar::all(0) :
            cv::mean(m);
  }

  if( output_map.needed() ) {

    if( m.empty() ) {
      output_map.create(src.size(),
          CV_MAKETYPE(CV_32F, avgchannel ? 1 : src.channels()));
    }
    else {

      if( uscale > 0 && uscale > dscale ) {
        downscale(m, m, uscale - std::max(0, dscale));
      }

      if( m.size() != src.size() ) {
        upscale(m, src.size());
      }

      output_map.move(m);
    }
  }


  return true;
}
