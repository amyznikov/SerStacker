/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC_IMAGE, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_BLUR1_IMAGE, "BLUR1", "" },
      { c_alpha_test_routine::DISPLAY_BLUR2_IMAGE, "BLUR2", "" },
      { c_alpha_test_routine::DISPLAY_TEXTURE_IMAGE, "TEXTURE", "" },
      { c_alpha_test_routine::DISPLAY_BLURED_TEXTURE, "BLURED_TEXTURE", "" },
      { c_alpha_test_routine::DISPLAY_K_IMAGE, "K", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE, "FILTERED", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE}
  };
  return members;
}



void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "sigma1", CTL_CONTEXT(ctx, _sigma1), "");
  ctlbind(ctls, "sigma2", CTL_CONTEXT(ctx, _sigma2), "");
  ctlbind(ctls, "sigma_se", CTL_CONTEXT(ctx, _sigma_se), "");
  ctlbind(ctls, "lpgk", CTL_CONTEXT(ctx, _lpgk), "");
  ctlbind(ctls, "tex_scale", CTL_CONTEXT(ctx, _tex_scale), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _sigma1);
    SERIALIZE_OPTION(settings, save, *this, _sigma2);
    SERIALIZE_OPTION(settings, save, *this, _sigma_se);
    SERIALIZE_OPTION(settings, save, *this, _tex_scale);
    SERIALIZE_OPTION(settings, save, *this, _lpgk);
    return true;
  }
  return false;
}

static void compute_gradient(const cv::Mat & src, cv::Mat & g)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 7, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  cv::Mat gx, gy;
  cv::sepFilter2D(src, gx, CV_32F, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, CV_32F, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(gx, gy, g);
}

static void compute_modlaplace(const cv::Mat & src, cv::Mat & l, double scale)
{
  static thread_local cv::Mat Kxx, Kyy;
  if( Kxx.empty() ) {
    cv::getDerivKernels(Kxx, Kyy, 2, 0, 7, true, CV_32F);
    Kxx *= M_SQRT2;
    Kyy *= M_SQRT2;
  }

  cv::Mat lx, ly;

  cv::sepFilter2D(src, lx, CV_32F, Kxx * scale, Kyy, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, ly, CV_32F, Kyy, Kxx * scale, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::magnitude(lx, ly, l);
}


static void compute_lpg(cv::InputArray _src, cv::OutputArray _dst, double gsigma, double lpgk)
{
  cv::Mat src, g, l;

  cv::GaussianBlur(_src, src, cv::Size(), gsigma, gsigma);
  compute_gradient(src, g);
  cv::GaussianBlur(g, g, cv::Size(), gsigma, gsigma);

  if ( lpgk > 0 ) {
    compute_modlaplace(src, l, lpgk);
    cv::add(l, g, _dst);
  }
  else {
    _dst.move(g);
  }
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  const int cn = image.channels();

  cv::Mat src, blur1, blur2, detail, gray, texture, K, filtered;

  if ( image.depth() == CV_32F ) {
    src = image.getMat();
  }
  else {
    image.getMat().convertTo(src, CV_32F);
  }

  if( cn == 1 ) {
    gray = src;
  }
  else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  if( _sigma1 > 0 ) {
    cv::GaussianBlur(src, blur1, cv::Size(), _sigma1, _sigma1);
  }
  else {
    blur1 = src;
  }

  if( _display == DISPLAY_BLUR1_IMAGE ) {
    image.move(blur1);
    return true;
  }

  if( _sigma2 > 0 ) {
    cv::GaussianBlur(blur1, blur2, cv::Size(), _sigma2, _sigma2);
  }
  else {
    blur2 = blur1;
  }

  if( _display == DISPLAY_BLUR2_IMAGE ) {
    image.move(blur2);
    return true;
  }

  compute_lpg(gray, texture, _sigma_se, _lpgk);

  if( _display == DISPLAY_TEXTURE_IMAGE ) {
    image.move(texture);
    return true;
  }

  if( _tex_scale > 0 ) {
    cv::multiply(texture, _tex_scale, texture);
  }

  if ( _display == DISPLAY_BLURED_TEXTURE ) {
    image.move(texture);
    return true;
  }

  cv::divide(1.0, texture + 1, K);
  if ( _display == DISPLAY_K_IMAGE ) {
    image.move(K);
    return true;
  }

  if( cn > 1 ) {
    cv::cvtColor(texture, texture, cv::COLOR_GRAY2BGR);
    cv::cvtColor(K, K, cv::COLOR_GRAY2BGR);
  }

  cv::multiply(texture.mul(blur1) + blur2, K, filtered);
  image.move(filtered);

  return true;
}

//bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  if ( _display == DISPLAY_SRC_IMAGE ) {
//    return true;
//  }
//
//  cv::Mat src, blur1, blur2, detail, gray, ramplee, K, filtered;
//
//  const int cn = image.channels();
//
//  if ( image.depth() == CV_32F ) {
//    src = image.getMat();
//  }
//  else {
//    image.getMat().convertTo(src, CV_32F);
//  }
//
//  if ( cn == 1 ) {
//    gray = src;
//  }
//  else {
//    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
//  }
//
//  if (_sigma1 > 0 ) {
//    cv::GaussianBlur(src, blur1, cv::Size(), _sigma1, _sigma1);
//  }
//  else {
//    blur1 = src;
//  }
//
//  if ( _display == DISPLAY_BLUR1_IMAGE ) {
//    image.move(blur1);
//    return true;
//  }
//
//  if (_sigma2 > 0 ) {
//    cv::GaussianBlur(blur1, blur2, cv::Size(), _sigma2, _sigma2);
//  }
//  else {
//    blur2 = blur1;
//  }
//
//  if ( _display == DISPLAY_BLUR2_IMAGE ) {
//    image.move(blur2);
//    return true;
//  }
//
//  const cv::Size se_size(2 * _se_radius + 1, 2 * _se_radius + 1);
//  const cv::Mat SE = cv::getStructuringElement(cv::MORPH_ELLIPSE, se_size, cv::Point(-1, -1));
//  rampLee(gray, ramplee, SE, cv::BORDER_DEFAULT);
//  if ( _display == DISPLAY_RAMPLEE_IMAGE ) {
//    image.move(ramplee);
//    return true;
//  }
//
//  morphological_smooth_close(ramplee, ramplee, cv::Mat1b(3,3, 255));
//
//  if ( _sigma_se > 0 ) {
//    cv::GaussianBlur(ramplee, ramplee, cv::Size(), _sigma_se, _sigma_se);
//  }
//
//  cv::multiply(ramplee, ramplee, ramplee);
//
//  if ( _ramplee_scale > 0 ) {
//    cv::multiply(ramplee, _ramplee_scale, ramplee);
//  }
//
//
//  if ( _display == DISPLAY_RAMPLEE_BLURED_IMAGE ) {
//    image.move(ramplee);
//    return true;
//  }
//
//  cv::add(ramplee, 1, K);
//
//  if ( _display == DISPLAY_RK_IMAGE ) {
//    cv::divide(ramplee, K, image);
//    return true;
//  }
//
//  if ( cn > 1 ) {
//    cv::cvtColor(ramplee, ramplee, cv::COLOR_GRAY2BGR);
//    cv::cvtColor(K, K, cv::COLOR_GRAY2BGR);
//  }
//
//  cv::divide(ramplee.mul(blur1) + blur2, K, filtered);
//  image.move(filtered);
//
//  return true;
//}



//bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  if ( _display == DISPLAY_SRC_IMAGE ) {
//    return true;
//  }
//
//  cv::Mat src, gray, texlee, ramplee, totaltex, blur1, blur2, filtered;
//
//  const int cn = image.channels();
//
//  if ( image.depth() == CV_32F ) {
//    src = image.getMat();
//  }
//  else {
//    image.getMat().convertTo(src, CV_32F);
//  }
//
//  if ( cn == 1 ) {
//    gray = src;
//  }
//  else {
//    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
//  }
//
//  const cv::Size se1_size(2 * _se_radius + 1, 2 * _se_radius + 1);
//  const cv::Mat SE1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, se1_size, cv::Point(-1, -1));
//  rampLee(gray, ramplee, SE1, cv::BORDER_DEFAULT);
//  if ( _display == DISPLAY_RAMPLEE_IMAGE ) {
//    image.move(ramplee);
//    return true;
//  }
//
//
//  const cv::Size se2_size(4 * _se_radius + 1, 4 * _se_radius + 1);
//  const cv::Mat SE2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, se2_size, cv::Point(-1, -1));
//  texLee(gray, texlee, SE2, cv::BORDER_DEFAULT);
//  if ( _display == DISPLAY_TEXLEE_IMAGE ) {
//    image.move(texlee);
//    return true;
//  }
//
//  if ( _sigma_se > 0 ) {
//    cv::GaussianBlur(texlee, texlee, cv::Size(), _sigma_se, _sigma_se);
//    cv::GaussianBlur(ramplee, ramplee, cv::Size(), _sigma_se, _sigma_se);
//  }
//  if ( _ramplee_scale > 0  ) {
//    cv::multiply(ramplee, _ramplee_scale, ramplee);
//  }
//
//  cv::add(texlee, ramplee, totaltex);
//  cv::divide(texlee, totaltex, texlee);
//  cv::divide(ramplee, totaltex, ramplee);
//
//  if ( _display == DISPLAY_TEXLEE_BLURED_IMAGE ) {
//    image.move(texlee);
//    return true;
//  }
//
//  if ( _display == DISPLAY_RAMPLEE_BLURED_IMAGE ) {
//    image.move(ramplee);
//    return true;
//  }
//
//  if ( _display == DISPLAY_TOTALTEX_IMAGE ) {
//    image.move(totaltex);
//    return true;
//  }
//
//  if (_sigma1 > 0 ) {
//    cv::GaussianBlur(src, blur1, cv::Size(), _sigma1, _sigma1);
//  }
//  else {
//    blur1 = src;
//  }
//
//  if ( _display == DISPLAY_BLUR1_IMAGE ) {
//    image.move(blur1);
//    return true;
//  }
//
//  if (_sigma2 > 0 ) {
//    cv::GaussianBlur(src, blur2, cv::Size(), _sigma2, _sigma2);
//  }
//  else {
//    blur2 = src;
//  }
//
//  if ( _display == DISPLAY_BLUR2_IMAGE ) {
//    image.move(blur2);
//    return true;
//  }
//
//
//  if ( cn > 1 ) {
//    cv::cvtColor(texlee, texlee, cv::COLOR_GRAY2BGR);
//    cv::cvtColor(ramplee, ramplee, cv::COLOR_GRAY2BGR);
//    cv::cvtColor(totaltex, totaltex, cv::COLOR_GRAY2BGR);
//  }
//
//  cv::add(ramplee.mul(blur1), texlee.mul(blur2), filtered);
//  image.move(filtered);
//
//  return true;
//}
//
//
