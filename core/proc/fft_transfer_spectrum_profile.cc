/*
 * fft_transfer_spectrum_profile.cc
 *
 *  Created on: Oct 27, 2020
 *      Author: amyznikov
 */

#include "fft_transfer_spectrum_profile.h"
#include "fft.h"
#include <core/debug.h>

static void compute_module_ratio(const cv::Mat & _spec1, const cv::Mat & _spec2, cv::Mat1f & ratio )
{
  typedef std::complex<float> complex;

  const cv::Mat_<complex> spec1 = _spec1;
  const cv::Mat_<complex> spec2 = _spec2;

  const double eps = FLT_EPSILON;// / spec1.size().area();

  ratio.create(spec1.size());
  for ( int y = 0; y < spec1.rows; ++y ) {
    for ( int x = 0; x < spec1.cols; ++x ) {
      const double m1 = log(eps + std::abs(spec1[y][x]));
      const double m2 = log(eps + std::abs(spec2[y][x]));
      ratio[y][x] = m1 - m2;
    }
  }

  fftSwapQuadrants(ratio);

  cv::GaussianBlur(ratio, ratio, cv::Size(0, 0),
      std::max(1., std::max(ratio.cols, ratio.rows) / 200.), 0,
      cv::BORDER_REPLICATE);

  fftSwapQuadrants(ratio);

  cv::exp(ratio, ratio);
}

static void multiply(cv::Mat & _spec, const cv::Mat1f & ratio)
{
  typedef std::complex<float> complex;
  cv::Mat_<complex> spec = _spec;

  for ( int y = 0; y < spec.rows; ++y ) {
    for ( int x = 0; x < spec.cols; ++x ) {
      spec[y][x] *= ratio[y][x];
    }
  }
}


bool fft_transfer_spectrum_profile(cv::InputArray from_image,
    cv::InputArray to_image,
    cv::OutputArray dst)
{
  cv::Mat source_image, target_image;
  cv::Rect source_rc, target_rc;
  cv::Size source_size, target_size, fft_size;
  std::vector<double> radial_profile;

  cv::Mat1f ratio;

  source_size = from_image.size();
  target_size = to_image.size();

  if ( source_size != target_size ) {
    CF_ERROR("ERROR: image sizes not equal: source_size=%dx%d target_size=%dx%d",
        source_size.width, source_size.height,
        target_size.width, target_size.height);
    return false;
  }

  fft_size = getOptimalFFTSize(source_size);

  if ( from_image.channels() == 1 ) {
    source_image = from_image.getMat();
  }
  else {
    cv::cvtColor(from_image,
        source_image,
        cv::COLOR_BGR2GRAY);
  }

  if ( to_image.channels() == 1 ) {
    target_image = to_image.getMat();
  }
  else {
    cv::cvtColor(to_image,
        target_image,
        cv::COLOR_BGR2GRAY);
  }




  if ( source_size != fft_size ) {
    copyMakeFFTBorder(source_image,
        source_image,
        fft_size,
        &source_rc);
  }
  cv::dft(source_image,
      source_image,
      cv::DFT_COMPLEX_OUTPUT);




  if ( target_size != fft_size ) {
    copyMakeFFTBorder(target_image,
        target_image,
        fft_size,
        &target_rc);
  }
  cv::dft(target_image,
      target_image,
      cv::DFT_COMPLEX_OUTPUT);




  compute_module_ratio(source_image,
      target_image,
      ratio);

  const int cn = to_image.channels();
  if ( cn == 1 ) {

    multiply(target_image,
        ratio);

    cv::idft(target_image,
        target_image,
        cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);

  }
  else {

    cv::Mat channels[cn];

    cv::split(to_image.getMat(),
        channels);

    for ( int i = 0; i < cn; ++i ) {

      if ( target_size != fft_size ) {
        copyMakeFFTBorder(channels[i],
            channels[i],
            fft_size);
      }

      cv::dft(channels[i],
          channels[i],
          cv::DFT_COMPLEX_OUTPUT);

      multiply(channels[i],
          ratio);

      cv::idft(channels[i],
          channels[i],
          cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);

    }

    cv::merge(channels, cn,
        target_image);
  }

  if ( target_rc.empty() ) {
    dst.move(target_image);
  }
  else {
    target_image(target_rc).copyTo(dst);
  }

  return true;
}


bool accumulate_fft_spectrum_power(const cv::Mat & src,  cv::Mat & acc, float & cnt)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Size fft_size;
  cv::Mat img, spec;
  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() && cnt > 0 ) {
    fft_size = acc.size();
  }
  else {
    fft_size = getOptimalFFTSize(src.size());
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
    cnt = 0;
  }

  copyMakeFFTBorder(src, img, fft_size, nullptr);

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }

  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }

  cv::add(acc, img, acc);
  ++cnt;

  return true;
}

bool max_fft_spectrum_power(const cv::Mat & src, cv::Mat & acc)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Size fft_size;
  cv::Mat img, spec;

  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() ) {
    fft_size = acc.size();
  }
  else {
    fft_size = getOptimalFFTSize(src.size());
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
  }

  if ( !copyMakeFFTBorder(src, img, fft_size, nullptr) ) {
    CF_ERROR("copyMakeFFTBorder(src=%dx%d, img, fft_size=%dx%d) fails", src.cols, src.rows, fft_size.width, fft_size.height);
    return false;
  }

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }


  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }

  cv::max(acc, img, acc);

  return true;
}


bool swap_fft_power_spectrum(const cv::Mat & src, const cv::Mat & acc, cv::Mat & dst)
{
  if ( acc.depth() != CV_32F ) {
    CF_ERROR("Invalid argument: acc.depth()=CV_32F is expected");
    return false;
  }

  if ( acc.channels() != src.channels() ) {
    CF_ERROR("Invalid argument: number of channels in src and sp not match");
    return false;
  }

  typedef std::complex<float> complex;

  cv::Mat img, spec;
  cv::Rect rc;
  const cv::Size fft_size = acc.size();
  const int cn = src.channels();
  cv::Mat channels[cn];


  if ( !copyMakeFFTBorder(src, img, fft_size, &rc) ) {
    CF_ERROR("copyMakeFFTBorder() fails");
    return false;
  }

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }


  for ( int i = 0; i < cn; ++i ) {

    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);

    cv::Mat_<complex> spec = channels[i];

    for ( int y = 0; y < spec.rows; ++y ) {

      const float * accp = acc.ptr<const float>(y);

      for ( int x = 0; x < spec.cols; ++x ) {

        spec[y][x] = std::polar(accp[x * cn + i], std::arg(spec[y][x]));

      }
    }

    cv::idft(channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }

  img(rc).copyTo(dst);

  return true;
}
