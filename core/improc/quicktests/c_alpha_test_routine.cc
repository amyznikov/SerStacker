/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC_IMAGE, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_BLURRED_IMAGE, "BLURRED", "" },
      { c_alpha_test_routine::DISPLAY_K_IMAGE, "K", "" },
      { c_alpha_test_routine::DISPLAY_DETAIL_IMAGE, "DETAIL", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE, "FILTERED", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE}
  };
  return members;
}

static void adaptive_gaussian_blur(cv::InputArray _src, cv::OutputArray _dst,
    int radius, double max_sigma, double noise_std, bool _sqrt,
    cv::OutputArray outputBlurred = cv::noArray(),
    cv::OutputArray outputK = cv::noArray(),
    cv::OutputArray outputDetail = cv::noArray())
{
  const int ddepth = _dst.fixedType() ? _dst.depth() : _src.depth();
  const int cn = _src.channels();

  cv::Mat src, gray;
  cv::Mat m, s, K;

  if ( _src.depth() == CV_32F ) {
    src = _src.getMat();
  }
  else {
    _src.getMat().convertTo(src, CV_32F);
  }

  if( cn == 1 ) {
    gray = src;
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  // local variance: Var = E[X^2] - (E[X])^2
  const cv::Size ksize(2 * radius + 1, 2 * radius + 1);
  cv::GaussianBlur(gray, m, ksize, 0, 0, cv::BORDER_REPLICATE);
  cv::GaussianBlur(gray.mul(gray), s, ksize, 0, 0, cv::BORDER_REPLICATE);
  cv::absdiff(s, m.mul(m), s);
  if ( _sqrt ) {
    cv::sqrt(s, s);
  }
  cv::divide(s, s + (_sqrt ? noise_std : noise_std * noise_std), K);
  if ( cn != 1 ) {
    cv::cvtColor(K, K, cv::COLOR_GRAY2BGR);
  }

  // Basic Gaussian blur for the entire image
  if ( _dst.needed() || outputDetail.needed() || outputBlurred.needed() ) {
    cv::Mat blurred, detail;
    cv::GaussianBlur(src, blurred, ksize, max_sigma, max_sigma, cv::BORDER_REPLICATE);

    if ( outputBlurred.needed() ) {
      blurred.copyTo(outputBlurred);
    }

    cv::subtract(src, blurred, detail);
    cv::multiply(K, detail, detail);

    if ( outputDetail.needed() ) {
      detail.copyTo(outputDetail);
    }

    cv::add(blurred, detail, detail);
    if ( ddepth == detail.depth() ) {
      _dst.move(detail);
    }
    else {
      detail.convertTo(_dst, ddepth);
    }
  }

  if ( outputK.needed() ) {
    outputK.move(K);
  }
}


void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "radius", CTL_CONTEXT(ctx, _radius), "");
  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, _sigma), "");
  ctlbind(ctls, "noise_std", CTL_CONTEXT(ctx, _noise_std), "");
  ctlbind(ctls, "sqrt", CTL_CONTEXT(ctx, _sqrt), "");

}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _radius);
    SERIALIZE_OPTION(settings, save, *this, _sigma);
    SERIALIZE_OPTION(settings, save, *this, _noise_std);
    SERIALIZE_OPTION(settings, save, *this, _sqrt);
    return true;
  }
  return false;
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  const cv::Mat src = image.getMat();

  cv::Mat filterd, blurred, K, detail;

  adaptive_gaussian_blur(src, filterd,
      _radius, _sigma, _noise_std, _sqrt,
      blurred, K, detail);

  if ( _display == DISPLAY_K_IMAGE ) {
    image.move(K);
    return true;
  }

  if ( _display == DISPLAY_DETAIL_IMAGE ) {
    image.move(detail);
    return true;
  }

  if ( _display == DISPLAY_BLURRED_IMAGE ) {
    image.move(blurred);
    return true;
  }


  image.move(filterd);

  return true;
}


