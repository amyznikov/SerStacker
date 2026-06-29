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
      { c_fft_filter_routine::FILTER_BUTTERWORTH, "BUTTERWORTH", },
      { c_fft_filter_routine::FILTER_GAUSSIAN_UNSHARP, "GAUSSIAN_UNSHARP", },
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
      { c_fft_filter_routine::DISPLAY_VLAP, "VLAP" },
      { c_fft_filter_routine::DISPLAY_FILTERED_IMAGE, },
  };
  return members;
}

void c_fft_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Filter: ", CTL_CONTEXT(ctx, _filterType), "Select filter type");
  ctlbind(ctls, "ppsDecomposition", CTL_CONTEXT(ctx, _ppsDecomposition), "");

  ctlbind_expandable_group(ctls, "Gaussian filter options",
      [&, ctx = CTL_CONTEXT(ctx, gaussian)]() {
        ctlbind(ctls, "sigma [px]: ", CTL_CONTEXT(ctx, sigma), "Gaussian blur sigma");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Laplacian filter options",
      [&, ctx = CTL_CONTEXT(ctx, laplacian)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Ramp filter options",
      [&, ctx = CTL_CONTEXT(ctx, ramp)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Butterworth filter options",
      [&, ctx = CTL_CONTEXT(ctx, butterworth)]() {
        ctlbind(ctls, "rc [pix]: ", CTL_CONTEXT(ctx, rc), "Butterworth cutoff in image space domain:\n FILTER = 1.0 / (1.0 + (r / rc)^(order))");
        ctlbind(ctls, "order: ", CTL_CONTEXT(ctx, order), "Butterworth filter order:\n FILTER = 1.0 / (1.0 + (r / rc)^(order))");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Gaussian unsharp filter options",
      [&, ctx = CTL_CONTEXT(ctx, gaussian_unsharp)]() {
        ctlbind(ctls, "sigma [px]: ", CTL_CONTEXT(ctx, sigma), "Gaussian unsharp sigma in image space domain");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

}

bool c_fft_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _filterType);
    SERIALIZE_OPTION(settings, save, *this, _ppsDecomposition);

    if ( auto group = SERIALIZE_GROUP(settings, save, "GaussianFilter")) {
      SERIALIZE_OPTION(settings, save, gaussian, sigma);
      SERIALIZE_OPTION(settings, save, gaussian, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "LaplacianFilter")) {
      SERIALIZE_OPTION(settings, save, laplacian, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "GradientFilter")) {
      SERIALIZE_OPTION(settings, save, ramp, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "ButterworthFilter")) {
      SERIALIZE_OPTION(settings, save, butterworth, rc);
      SERIALIZE_OPTION(settings, save, butterworth, order);
      SERIALIZE_OPTION(settings, save, butterworth, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "GaussianUnsharpFilter")) {
      SERIALIZE_OPTION(settings, save, gaussian_unsharp, sigma);
      SERIALIZE_OPTION(settings, save, gaussian_unsharp, gain);
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
      const int ksize = std::max(3, std::min(63, 2 * int(3 * gaussian.sigma) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateGaussianFilter(fftSize, gaussian.sigma, gaussian.gain);
      break;
    }

    case FILTER_LAPLACIAN: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateLaplacianFilter(fftSize, laplacian.gain);
      break;
    }

    case FILTER_RAMP: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateRampFilter(fftSize, ramp.gain);
      break;
    }

    case FILTER_BUTTERWORTH: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateButterworthFilter(fftSize, butterworth.rc, butterworth.order, butterworth.gain);
      break;
    }

    case FILTER_GAUSSIAN_UNSHARP: {
      const int ksize = std::max(3, std::min(63, 2 * int(3 * gaussian_unsharp.sigma) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateGaussianUnsharpFilter(fftSize, gaussian_unsharp.sigma, gaussian_unsharp.gain);
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

  if( !_ppsDecomposition ) {
    VLAP.release();
  }
  else if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
  }

  if ( _display == DISPLAY_VLAP ) {
    // No further processing requested
    mask.release();
    return fftDisplay(VLAP, image);
  }



  std::vector<cv::Mat> real_channels(cn);
  std::vector<cv::Mat> complex_channels(cn);
  std::vector<cv::Mat> complex_channels_s(cn);
  cv::split(src, real_channels);

  for ( int i = 0; i < cn; ++i ) {

    fftCopyMakeBorder(real_channels[i], real_channels[i], fftSize);
    real_channels[i].convertTo(real_channels[i], CV_32F);

    if ( ! _ppsDecomposition ) {
      fftImageToSpectrum(real_channels[i], complex_channels[i], fftSize);
    }
    else {
      fftPPSDecomposition(real_channels[i], VLAP, complex_channels[i], complex_channels_s[i]);
    }

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
    if ( _ppsDecomposition ) {
      cv::add(complex_channels[i], complex_channels_s[i], complex_channels[i]);
    }

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

