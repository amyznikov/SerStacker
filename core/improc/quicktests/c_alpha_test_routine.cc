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
      { c_alpha_test_routine::DISPLAY_LAP_IMAGE, "LAP", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE, "FILTERED", "" },
      { c_alpha_test_routine::DISPLAY_FILTERED_IMAGE}
  };
  return members;
}


void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, _sigma), "");
//  ctlbind(ctls, "scale", CTL_CONTEXT(ctx, _scale), "");
  ctlbind(ctls, "target_snr", CTL_CONTEXT(ctx, _target_snr), "");

}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _sigma);
//    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _target_snr);

    return true;
  }
  return false;
}

//static double get_stdev(cv::InputArray src, cv::InputArray mask)
//{
//  cv::Scalar mean, stddev;
//  cv::meanStdDev(src, mean, stddev);
//
//  const int cn = src.channels();
//  double s = 0;
//  for( int c = 0; c < cn; ++c ) {
//    s += stddev[c] * stddev[c];
//  }
//
//  return std::sqrt(s / cn);
//}
//
//static inline double square(double x)
//{
//  return x * x;
//}

static inline double compute_noise(cv::InputArray src, cv::InputArray mask)
{
  const int cn = src.channels();
  const cv::Scalar noise = estimate_noise(src, cv::noArray(), mask);
  double s = noise[0];
  for ( int c = 1; c < cn; ++c ) {
    s += noise[c];
  }
  return s / cn;
}

double estimate_invariant_usm_snr(const cv::Mat & csrc,
    const cv::Mat & cblur,
    double noise_stddev,
    double blur_sigma,
    const cv::Mat & mask)
{
  if (csrc.empty() || noise_stddev <= 0.0 || blur_sigma <= 0.5) {
    return 0.0;
  }

  cv::Mat usm_band = csrc - cblur;

  double noise_variance = noise_stddev * noise_stddev;
  double noise_factor = 1.0 - (1.0 / (2.0 * std::sqrt(CV_PI) * blur_sigma));
  double expected_noise_var_usm = noise_variance * noise_factor;

  int min_dim = std::min(csrc.rows, csrc.cols);
  int ksize = std::max(static_cast<int>(blur_sigma * 4.0), min_dim / 64) | 1;
  const cv::Size block_size(ksize, ksize);

  cv::Mat mean_U, mean_U2;
  cv::boxFilter(usm_band, mean_U, CV_32F, block_size);
  cv::boxFilter(usm_band.mul(usm_band), mean_U2, CV_32F, block_size);

  cv::Mat local_variance = mean_U2 - mean_U.mul(mean_U);

  cv::Mat smoothed_variance;
  cv::blur(local_variance, smoothed_variance, block_size);

  cv::Mat valid_mask;
  if (!mask.empty()) {
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, block_size);
    cv::erode(mask, valid_mask, element);
  }

  double max_local_variance = 0.0;
  cv::minMaxLoc(smoothed_variance, nullptr, &max_local_variance, nullptr, nullptr, valid_mask);

  double signal_variance = max_local_variance - expected_noise_var_usm;
  if( signal_variance <= 0.0 ) {
    return -20.0;
  }

  double pure_ratio = (signal_variance / expected_noise_var_usm) / blur_sigma;

  return 10.0 * std::log10(pure_ratio);
}


bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  const cv::Mat src = image.getMat();
  const cv::Mat maskMat = mask.getMat();
  const double blur_sigma = _sigma;

  const double noise_stddev = compute_noise(src, maskMat);
  if (noise_stddev <= 0.0 || blur_sigma <= 0.5) {
    return true;
  }

  cv::Mat csrc, cblur;
  src.convertTo(csrc, CV_32F);
  cv::GaussianBlur(csrc, cblur, cv::Size(), blur_sigma, blur_sigma, cv::BORDER_REPLICATE);

  const double invariant_snr = estimate_invariant_usm_snr(csrc, cblur, noise_stddev, blur_sigma, maskMat);

  const double target_snr = _target_snr;

    const double ALPHA_MAX_LIMIT = 100.0;
    double alpha_final = ALPHA_MAX_LIMIT;

    if (invariant_snr < target_snr) {
      double db_diff = target_snr - invariant_snr;
      double attenuation = std::pow(10.0, db_diff / 20.0);
      alpha_final = ALPHA_MAX_LIMIT / attenuation;
    }

    if (alpha_final < 1.0) {
      alpha_final = 1.0;
    }

    double w = 1.0 - (1.0 / alpha_final);
    w = std::clamp(w, 0.0, 0.99999);

  const double alpha = 1. / (1. - w);
  const double beta = -w / (1. - w);

  cv::addWeighted(csrc, alpha, cblur, beta, 0, image);

  const double noise2 = compute_noise(image, maskMat);

  CF_DEBUG("\n"
      "==> sigma=%g INV_SNR=%g noise1=%g noise2=%g _target_snr=%g w_adapted=%g alpha=%g",
      _sigma, invariant_snr, noise_stddev, noise2, _target_snr, w, alpha);

  return true;
}


