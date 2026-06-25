/*
 * c_fft_gaussian_filter_routine.cc
 *
 *  Created on: Jun 11, 2026
 *      Author: amyznikov
 */

#include "c_fft_filter_routine.h"

#include <core/proc/fft.h>
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_fft_filter_routine::FILTER>()
{
  static const c_enum_member members[] = {
      { c_fft_filter_routine::FILTER_GAUSSIAN, "GAUSSIAN", },
      { c_fft_filter_routine::FILTER_LAPLACIAN, "LAPLACIAN", },
      { c_fft_filter_routine::FILTER_RAMP, "RAMP", },
      { c_fft_filter_routine::FILTER_GAUSSIAN, },

  };
  return members;
}

template<>
const c_enum_member* members_of<c_fft_filter_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_filter_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { c_fft_filter_routine::DISPLAY_FILTERED_IMAGE, "FILTERED_IMAGE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_MODULE, "SRC_SPECTRUM_MODULE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_POWER, "SRC_SPECTRUM_POWER" },
      { c_fft_filter_routine::DISPLAY_FILTER_MODULE, "FILTER_MODULE" },
      { c_fft_filter_routine::DISPLAY_FILTER_POWER, "FILTER_POWER" },
      { c_fft_filter_routine::DISPLAY_FILTERED_SPECTRUM_MODULE, "FILTERED_SPECTRUM_MODULE" },
      { c_fft_filter_routine::DISPLAY_FILTERED_SPECTRUM_POWER, "FILTERED_SPECTRUM_POWER" },
      { c_fft_filter_routine::DISPLAY_FILTERED_IMAGE, },
  };
  return members;
}

void c_fft_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Filter: ", CTL_CONTEXT(ctx, _filterType), "Select filter type");

  ctlbind_expandable_group(ctls, "Gaussian filter options",
      [&, ctx = CTL_CONTEXT(ctx, gaussian_filter)]() {
        ctlbind(ctls, "sigma: ", CTL_CONTEXT(ctx, sigma), "Gaussian blur sigma");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Laplacian filter options",
      [&, ctx = CTL_CONTEXT(ctx, laplacian_filter)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Gradient filter options",
      [&, ctx = CTL_CONTEXT(ctx, ramp_filter)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

}

bool c_fft_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _filterType);

    if ( auto group = SERIALIZE_GROUP(settings, save, "GaussianFilter")) {
      SERIALIZE_OPTION(settings, save, gaussian_filter, sigma);
      SERIALIZE_OPTION(settings, save, gaussian_filter, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "LaplacianFilter")) {
      SERIALIZE_OPTION(settings, save, laplacian_filter, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "GradientFilter")) {
      SERIALIZE_OPTION(settings, save, ramp_filter, gain);
    }

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

//static bool fftMulSpectrum(cv::InputArray filter, cv::InputArray complexSpectrum,
//    cv::OutputArray dst)
//{
//  if( filter.type() != CV_32FC1 ) {
//    CF_ERROR("Invalid argument: Single channel CV_32F filter matrix is expected on input");
//    return false;
//  }
//
//  if( complexSpectrum.type() != CV_32FC2 ) {
//    CF_ERROR("Invalid argument: Two channel CV_32F complex spectrum matrix is expected on input");
//    return false;
//  }
//
//  cv::Mat2f F;
//  const cv::Mat planes[] {
//      filter.getMat(), filter.getMat()
//  };
//  cv::merge(planes, 2, F);
//  cv::multiply(F, complexSpectrum, dst);
//
//  return true;
//}


bool c_fft_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true; // No processing requested
  }

  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const int cn = src.channels();
  cv::Size fftSize;
  cv::Mat1f FILTER;


  switch (_filterType) {
    case FILTER_GAUSSIAN: {
      const int ksize = std::max(3, std::min(63, 2 * int(3 * gaussian_filter.sigma) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateGaussianFilter(fftSize, gaussian_filter.sigma, gaussian_filter.gain, true);
      break;
    }

    case FILTER_LAPLACIAN: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateLaplacianFilter(fftSize, laplacian_filter.gain, true);
      break;
    }

    case FILTER_RAMP: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateRampFilter(fftSize, ramp_filter.gain, true);
      break;
    }

    default:
      CF_ERROR("Not supported filter=%d requested", _filterType);
      break;
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

