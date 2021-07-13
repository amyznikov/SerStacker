/*
 * c_average_fft.cc
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#include "c_average_fft.h"
#include <core/proc/fft.h>
#include <core/debug.h>
#include <tbb/tbb.h>


static int countNaNs(const cv::Mat & image)
{
  int cnt = 0;

  const int nc = image.channels();

  for ( int y = 0; y < image.rows; ++y ) {

    const float * p = image.ptr<const float>(y);

    for ( int x = 0; x < image.cols * nc; ++x ) {
      if ( isnan(p[x]) ) {
        ++cnt;
      }
    }
  }

  return cnt;
}

static inline double power(double x)
{
  return x * x * x;
}

static inline double square(double x)
{
  return x * x;
}

static bool fftPower(const cv::Mat & src, cv::Mat & dst, bool mc )
{
  if ( src.channels() != 2 || src.depth() != CV_32F ) {
    CF_ERROR("invalid arg: FP32 2-channel input image expected");
    return false;
  }

  const cv::Mat2f csrc = src;
  cv::Mat2f cmag;

  cmag.create(src.size());

  double scale = square(1. / src.size().area());
  CF_DEBUG("scale=%g", scale);


  if ( !mc ) {
    for ( int y = 0; y < csrc.rows; ++y ) {
      for ( int x = 0; x < csrc.cols; ++x ) {
        const double a = csrc[y][x][0];
        const double b = csrc[y][x][1];
        const double p = power((a * a + b * b) * scale);
        cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
      }
    }
  }
  else {

    tbb::parallel_for(0, csrc.rows,
        [&csrc, &cmag, scale](int y) {
          for ( int x = 0; x < csrc.cols; ++x ) {
            const double a = csrc[y][x][0];
            const double b = csrc[y][x][1];
            const double p = power((a * a + b * b) * scale);
            cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
          }
        });
  }

  dst = std::move(cmag);

  return true;
}


void c_average_fft::reset()
{
  nbframes_ = 0;
  accumulators_.clear();
  weights_.clear();
  rc_.x = rc_.y = rc_.width = rc_.height = 0;
  fftSize_.width =  fftSize_.height = 0;
  border_top_ = 0;
  border_bottom_ = 0;
  border_left_ = 0;
  border_right_ = 0;
}

bool c_average_fft::add(cv::InputArray src)
{
  const int nc = src.channels();

  if ( !accumulators_.empty() && accumulators_.size() != nc ) {
    CF_ERROR("Number of channels not match, expected %zu channel input image", accumulators_.size());
    return  false;
  }

  cv::Mat channels[nc];
  cv::Mat weights[nc];

  if ( nc == 1 ) {
    src.getMat().copyTo(channels[0]);
  }
  else {
    cv::split(src.getMat(), channels);
  }

  const cv::Size src_size = channels[0].size();

  if ( accumulators_.empty() ) {

    fftSize_ = getOptimalFFTSize(src_size, cv::Size(0,0), false);

    CF_DEBUG("src_size=%dx%d fftSize_=%dx%d",
        src_size.width, src_size.height,
        fftSize_.width, fftSize_.height);


    if ( src_size == fftSize_ ) {
      rc_.x = rc_.y = rc_.width = rc_.height = 0;
    }
    else {
      border_top_ = (fftSize_.height - src_size.height) / 2;
      border_bottom_ = (fftSize_.height - src_size.height - border_top_);
      border_left_ = (fftSize_.width - src_size.width) / 2;
      border_right_ = (fftSize_.width - src_size.width - border_left_);
      rc_ = cv::Rect(border_left_, border_top_, src.cols(), src.rows());

    }

  }

  if ( nc == 1 ) {
    if ( src_size == fftSize_ ) {
      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);
    }
    else {

      cv::copyMakeBorder(channels[0], channels[0],
          border_top_, border_bottom_,
          border_left_, border_right_,
          cv::BORDER_REFLECT);

      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);

      fftPower(channels[0], weights[0], true);
    }
  }
  else {

    tbb::parallel_for(0, nc,
        [this, src_size, &channels, &weights](int i ) {

          if ( src_size == fftSize_ ) {
            cv::dft(channels[i], channels[i],
                cv::DFT_COMPLEX_OUTPUT);
          }
          else {

            cv::Mat tmp;

            cv::copyMakeBorder(channels[i], tmp,
                border_top_, border_bottom_,
                border_left_, border_right_,
                cv::BORDER_REFLECT);

            channels[i] = tmp;

            cv::dft(channels[i], channels[i],
                cv::DFT_COMPLEX_OUTPUT);

          }

          fftPower(channels[i], weights[i], false);
        });
  }

  if ( accumulators_.empty() ) {

    accumulators_.resize(nc);
    weights_.resize(nc);

    for ( int i = 0; i < nc; ++i ) {

      accumulators_[i].create(channels[i].size(), channels[i].type());
      accumulators_[i].setTo(0);

      weights_[i].create(weights[i].size(), weights[i].type());
      weights_[i].setTo(0);
    }

  }


  for ( int i = 0; i < nc; ++i ) {

    double min, max;

    cv::minMaxLoc(weights[i], &min, &max);
    CF_DEBUG("weights      [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(weights[i]), min, max);

    cv::accumulateProduct(channels[i], weights[i], accumulators_[i]);
    cv::accumulate(weights[i], weights_[i]);
  }


  ++nbframes_;

  return true;
}

bool c_average_fft::average(cv::OutputArray avg, double scale, int ddepth) const
{
  const int nc = accumulators_.size();
  cv::Mat channels[nc];

  for ( int i = 0; i < nc; ++i ) {
    double min, max;

    cv::divide(accumulators_[i], weights_[i], channels[i], scale, ddepth);

    cv::minMaxLoc(channels[i], &min, &max);
    CF_DEBUG("channels     [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(channels[i]), min, max);


    cv::idft(channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    cv::minMaxLoc(channels[i], &min, &max);
    CF_DEBUG("channels     [i=%d]: countNaNs=%d min=%g max=%g", i, countNaNs(channels[i]), min, max);

  }

  if ( rc_.empty() ) {
    if ( nc == 1 ) {
      avg.move(channels[0]);
    }
    else {
      cv::merge(channels, nc, avg);
    }
  }
  else {
    if ( nc == 1 ) {
      channels[0](rc_).copyTo(avg);
    }
    else {
      cv::Mat tmp;
      cv::merge(channels, nc, tmp);
      tmp(rc_).copyTo(avg);
    }
  }

  return true;
}

