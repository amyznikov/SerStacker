/*
 * c_fft_gaussian_filter_routine.cc
 *
 *  Created on: Jun 11, 2026
 *      Author: amyznikov
 */

#include "c_fft_gaussian_filter_routine.h"
#include <core/proc/fft.h>
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_fft_gaussian_filter_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_gaussian_filter_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { c_fft_gaussian_filter_routine::DISPLAY_FILTERED_IMAGE, "FILTERED_IMAGE" },
      { c_fft_gaussian_filter_routine::DISPLAY_SRC_SPECTRUM_MODULE, "SRC_SPECTRUM_MODULE" },
      { c_fft_gaussian_filter_routine::DISPLAY_SRC_SPECTRUM_POWER, "SRC_SPECTRUM_POWER" },
      { c_fft_gaussian_filter_routine::DISPLAY_GAUSSIAN_MODULE, "GAUSSIAN_MODULE" },
      { c_fft_gaussian_filter_routine::DISPLAY_GAUSSIAN_POWER, "GAUSSIAN_POWER" },
      { c_fft_gaussian_filter_routine::DISPLAY_FILTERED_SPECTRUM_MODULE, "FILTERED_SPECTRUM_MODULE" },
      { c_fft_gaussian_filter_routine::DISPLAY_FILTERED_SPECTRUM_POWER, "FILTERED_SPECTRUM_POWER" },
      { c_fft_gaussian_filter_routine::DISPLAY_FILTERED_IMAGE, },
  };
  return members;
}

void c_fft_gaussian_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "gsigma: ", CTL_CONTEXT(ctx, _gsigma), "Gaussian blur sigma");
  ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, _gain), "");
  ctlbind(ctls, "gain_cutoff: ", CTL_CONTEXT(ctx, _gain_cutoff), "");
}

bool c_fft_gaussian_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _gain);
    SERIALIZE_OPTION(settings, save, *this, _gain_cutoff);
    return true;
  }
  return false;
}


// Magnitude: sqrt(Re^2 + Im^2)
static bool fftDisplay(cv::InputArray _spec, cv::OutputArray _dst, bool swapQuadrants = false)
{
  if( _spec.type() == CV_32FC1 ) {
    _spec.getMat().copyTo(_dst);
    if( swapQuadrants ) {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  if ( _spec.type() == CV_32FC2 ) {
    cv::Mat1f magnitude;
    fftSpectrumModule(_spec, _dst);
    if ( swapQuadrants )  {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  CF_ERROR("Invalid argument: Single or Two channel CV_32F complex image is expected on input");
  return false;
}

static bool fftMulSpectrum(cv::InputArray filter, cv::InputArray complexSpectrum,
    cv::OutputArray dst)
{
  if( filter.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: Single channel CV_32F filter matrix is expected on input");
    return false;
  }

  if( complexSpectrum.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: Two channel CV_32F complex spectrum matrix is expected on input");
    return false;
  }

  cv::Mat2f F;
  const cv::Mat planes[] {
      filter.getMat(), filter.getMat()
  };
  cv::merge(planes, 2, F);
  cv::multiply(F, complexSpectrum, dst);

  return true;
}


bool c_fft_gaussian_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true; // No processing requested
  }

  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const int cn = src.channels();
  const cv::Size fftSize = fftGetOptimalSize(src.size());

  // CF_DEBUG("fftSize= %dx%d", fftSize.width, fftSize.height);

  const cv::Mat1f GAUSSIAN =
      fftGenerateGaussianFilter(fftSize, _gsigma);

  if ( _display == DISPLAY_GAUSSIAN_MODULE ) {
    // No further processing requested
    mask.release();
    return fftDisplay(GAUSSIAN, image);
  }
  if ( _display == DISPLAY_GAUSSIAN_POWER ) {
    // No further processing requested
    mask.release();
    return fftDisplay(GAUSSIAN.mul(GAUSSIAN), image);
  }

  std::vector<cv::Mat> real_channels(cn);
  std::vector<cv::Mat> complex_channels(cn);
  cv::split(src, real_channels);

  for ( int i = 0; i < cn; ++i ) {
    fftCopyMakeBorder(real_channels[i], real_channels[i], fftSize);
    real_channels[i].convertTo(real_channels[i], CV_32F);
    fftImageToSpectrum(real_channels[i], complex_channels[i], fftSize, true);
    if ( _display == DISPLAY_SRC_SPECTRUM_MODULE ) {
      fftSpectrumModule(complex_channels[i], real_channels[i]);
      continue;
    }
    if ( _display == DISPLAY_SRC_SPECTRUM_POWER ) {
      fftSpectrumModule(complex_channels[i], real_channels[i]);
      cv::multiply(real_channels[i], real_channels[i], real_channels[i]);
      continue;
    }

    fftMulSpectrum(GAUSSIAN, complex_channels[i], complex_channels[i]);
    if ( _display == DISPLAY_FILTERED_SPECTRUM_MODULE ) {
      fftSpectrumModule(complex_channels[i], real_channels[i]);
      continue;
    }
    if ( _display == DISPLAY_FILTERED_SPECTRUM_POWER ) {
      fftSpectrumModule(complex_channels[i], real_channels[i]);
      cv::multiply(real_channels[i], real_channels[i], real_channels[i]);
      continue;
    }

    // DISPLAY_FILTERED_IMAGE
    fftSwapQuadrants(complex_channels[i]);
    cv::idft(complex_channels[i], real_channels[i], cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    if ( !rc.empty() ) {
      real_channels[i] = real_channels[i](rc);
    }
  }

  cv::merge(real_channels, image);
  if ( _display != DISPLAY_FILTERED_IMAGE ) {
    mask.release();
  }

  return true;
}




