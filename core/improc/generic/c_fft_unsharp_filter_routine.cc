/*
 * c_fft_unsharp_filter_routine.cc
 *
 *  Created on: Jun 13, 2026
 *      Author: amyznikov
 */

#include "c_fft_unsharp_filter_routine.h"
#include <core/proc/fft.h>
#include <core/ssprintf.h>


template<>
const c_enum_member* members_of<c_fft_unsharp_filter_routine::FILTER>()
{
  static const c_enum_member members[] = {
      { c_fft_unsharp_filter_routine::FILTER_GAUSSIAN, "GAUSSIAN", },
      { c_fft_unsharp_filter_routine::FILTER_BUTTERWORTH, "BUTTERWORTH" },
      { c_fft_unsharp_filter_routine::FILTER_GAUSSIAN, },
  };
  return members;
}

template<>
const c_enum_member* members_of<c_fft_unsharp_filter_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_unsharp_filter_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTERED_IMAGE, "FILTERED_IMAGE" },
      { c_fft_unsharp_filter_routine::DISPLAY_SRC_SPECTRUM_MODULE, "SRC_SPECTRUM_MODULE" },
      { c_fft_unsharp_filter_routine::DISPLAY_SRC_SPECTRUM_POWER, "SRC_SPECTRUM_POWER" },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTER_MODULE, "FILTER_MODULE" },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTER_POWER, "FILTER_POWER" },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTERED_SPECTRUM_MODULE, "FILTERED_SPECTRUM_MODULE" },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTERED_SPECTRUM_POWER, "FILTERED_SPECTRUM_POWER" },
      { c_fft_unsharp_filter_routine::DISPLAY_FILTERED_IMAGE, },
  };
  return members;
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

void c_fft_unsharp_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Filter: ", CTL_CONTEXT(ctx, _filter), "Select base filter");
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");

  ctlbind_expandable_group(ctls, "Gaussian",
      [&, ctx = CTL_CONTEXT(ctx, gaussian_filter)]() {
        ctlbind(ctls, "gsigma: ", CTL_CONTEXT(ctx, sigma), "Gaussian blur sigma");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "Filter gain");
      });

  ctlbind_expandable_group(ctls, "Butterworth",
      [&, ctx = CTL_CONTEXT(ctx, butterworth_filter)]() {
        ctlbind(ctls, "rc: ", CTL_CONTEXT(ctx, rc), "Butterworth cutoff radius");
        ctlbind(ctls, "order: ", CTL_CONTEXT(ctx, order), "Butterworth order");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "Filter gain");
      });
}

bool c_fft_unsharp_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _filter);
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _gain_cutoff);

    if ( auto group = SERIALIZE_GROUP(settings, save, "gaussian_filter") ) {
      SERIALIZE_OPTION(settings, save, gaussian_filter, sigma);
      SERIALIZE_OPTION(settings, save, gaussian_filter, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "butterworth_filter") ) {
      SERIALIZE_OPTION(settings, save, butterworth_filter, rc);
      SERIALIZE_OPTION(settings, save, butterworth_filter, order);
      SERIALIZE_OPTION(settings, save, butterworth_filter, gain);
    }

    return true;
  }
  return false;
}



bool c_fft_unsharp_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true; // No any processing requested
  }

  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const int cn = src.channels();
  cv::Size fftSize;
  cv::Mat1f FILTER;

  switch (_filter) {
    case FILTER_BUTTERWORTH: {
      const int ksize = std::min(63, std::max(3, 2 * (int) (3 * std::abs(butterworth_filter.rc)) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      //CF_DEBUG("fftSize= %dx%d", fftSize.width, fftSize.height);
      FILTER = fftGenerateButterworthUnsharpFilter(fftSize,
          butterworth_filter.rc,
          butterworth_filter.order,
          butterworth_filter.gain,
          true);
      break;
    }

    case FILTER_GAUSSIAN:
      default: {
      const int ksize = std::min(63, std::max(3, 2 * (int) (3 * std::abs(gaussian_filter.sigma)) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      //CF_DEBUG("fftSize= %dx%d", fftSize.width, fftSize.height);
      FILTER = fftGenerateGaussianUnsharpFilter(fftSize,
          gaussian_filter.sigma,
          gaussian_filter.gain,
          true);
      break;
    }
  }

  if ( _display == DISPLAY_FILTER_MODULE ) {
    // No further processing requested
    mask.release();
    return fftDisplay(FILTER, image);
  }
  if ( _display == DISPLAY_FILTER_POWER ) {
    // No further processing requested
    mask.release();
    return fftDisplay(FILTER.mul(FILTER), image);
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

    fftMulSpectrum(FILTER, complex_channels[i], complex_channels[i]);
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

