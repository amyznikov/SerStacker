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
  ctlbind(ctls, "includeCorners", CTL_CONTEXT(ctx, _includeCorners), "");
  ctlbind(ctls, "profileToImage", CTL_CONTEXT(ctx, _profileToImage), "");
}

bool c_fft_radial_profile_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _includeCorners);
    SERIALIZE_OPTION(settings, save, *this, _profileToImage);
    return true;
  }
  return false;

}

bool c_fft_radial_profile_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();
  const int cn = src.channels();
  const cv::Size fftSize = fftGetOptimalSize(src.size());


  std::vector<cv::Mat> channels(cn);
  cv::Mat1f radial_profile;
  cv::Rect rc;

  cv::split(src, channels);

  for ( int i = 0; i < cn; ++i ) {
    fftCopyMakeBorder(channels[i], channels[i], fftSize, &rc);
    channels[i].convertTo(channels[i], CV_32F);
    fftImageToSpectrum(channels[i], channels[i], fftSize, true);
    fftSpectrumModule(channels[i], channels[i]);
    if ( _profileToImage) {
      fftRadialProfile(channels[i], radial_profile, _includeCorners);
      fftRadialProfileToImage(radial_profile, fftSize, _includeCorners, (cv::Mat1f&) channels[i]);
    }
  }

  cv::merge(channels, image);
  mask.release();
  return true;
}

