/*
 * c_fft_radial_profile_routine.cc
 *
 *  Created on: Jun 9, 2026
 *      Author: amyznikov
 */

#include "c_fft_radial_profile_routine.h"
#include <core/io/c_stdio_file.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>

void c_fft_radial_profile_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "ppsDecomposition", CTL_CONTEXT(ctx, _ppsDecomposition), "");
  ctlbind(ctls, "profileToImage", CTL_CONTEXT(ctx, _profileToImage), "");
}

bool c_fft_radial_profile_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _ppsDecomposition);
    SERIALIZE_OPTION(settings, save, *this, _profileToImage);
    return true;
  }
  return false;

}

bool c_fft_radial_profile_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const int cn = src.channels();
  const cv::Size fftSize = fftGetOptimalSize(src.size(), cv::Size(0, 0), &rc);

  std::vector<cv::Mat> channels(cn);
  cv::Mat1f radial_profile;

  cv::split(src, channels);

  if( !_ppsDecomposition ) {
    VLAP.release();
  }
  else if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
  }

  for ( int i = 0; i < cn; ++i ) {
    fftCopyMakeBorder(channels[i], channels[i], fftSize);
    channels[i].convertTo(channels[i], CV_32F);

    if ( ! _ppsDecomposition ) {
      fftImageToSpectrum(channels[i], channels[i], fftSize, true);
    }
    else {
      fftPPSDecomposition(channels[i], VLAP, channels[i], cv::noArray(), true);
    }

    fftSpectrumModule(channels[i], channels[i]);
    if ( _profileToImage) {
      fftRadialProfile(channels[i], radial_profile);
      fftRadialProfileToImage(radial_profile, fftSize, (cv::Mat1f&) channels[i]);
    }
  }

  cv::merge(channels, image);
  mask.release();
  return true;
}

