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
      { c_fft_filter_routine::FILTER_BUTTERWORTH_BAND, "BUTTERWORTH_BAND", },
      { c_fft_filter_routine::FILTER_GAUSSIAN_SHARP, "GAUSSIAN_SHARP", },
      { c_fft_filter_routine::FILTER_LAPLACIAN_SHARP, "LAPLACIAN_SHARP", },
      // { c_fft_filter_routine::FILTER_LAPLACIAN_LPASS, "LAPLACIAN_LPASS", },

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
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_PHASE, "SRC_SPECTRUM_PHASE" },
      { c_fft_filter_routine::DISPLAY_FILTER_MODULE, "FILTER_MODULE" },
      { c_fft_filter_routine::DISPLAY_FILTERED_SPECTRUM_MODULE, "FILTERED_SPECTRUM_MODULE" },
      { c_fft_filter_routine::DISPLAY_FILTERED_SPECTRUM_PHASE, "FILTERED_SPECTRUM_PHASE" },
      { c_fft_filter_routine::DISPLAY_VLAP, "VLAP" },

      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_P_MODULE, "SRC_SPECTRUM_P_MODULE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_P_PHASE, "SRC_SPECTRUM_P_PHASE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_S_MODULE, "SRC_SPECTRUM_S_MODULE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_S_PHASE, "SRC_SPECTRUM_S_PHASE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_V_MODULE, "SRC_SPECTRUM_V_MODULE" },
      { c_fft_filter_routine::DISPLAY_SRC_SPECTRUM_V_PHASE, "SRC_SPECTRUM_V_PHASE" },

      { c_fft_filter_routine::DISPLAY_FILTERED_IMAGE, },
  };
  return members;
}

void c_fft_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Filter: ", CTL_CONTEXT(ctx, _filterType), "Select filter type");
  ctlbind(ctls, "ppsDecomposition", CTL_CONTEXT(ctx, _ppsDecomposition), "");
  ctlbind(ctls, "swapQuadrants: ", CTL_CONTEXT(ctx, _swapQuadrants), "swapQuadrants for spectrum displays");

  ctlbind_expandable_group(ctls, "Gaussian options",
      [&, ctx = CTL_CONTEXT(ctx, gaussian)]() {
        ctlbind(ctls, "sigma [px]: ", CTL_CONTEXT(ctx, sigma), "Gaussian blur sigma");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Laplacian options",
      [&, ctx = CTL_CONTEXT(ctx, laplacian)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Ramp options",
      [&, ctx = CTL_CONTEXT(ctx, ramp)]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Butterworth options",
      [&, ctx = CTL_CONTEXT(ctx, butterworth)]() {
        ctlbind(ctls, "rc [pix]: ", CTL_CONTEXT(ctx, rc), "Butterworth cutoff in image space domain:\n FILTER = 1.0 / (1.0 + (r / rc)^(order))");
        ctlbind(ctls, "order: ", CTL_CONTEXT(ctx, order), "Butterworth filter order:\n FILTER = 1.0 / (1.0 + (r / rc)^(order))");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "ButterBabd options",
      [&, ctx = CTL_CONTEXT(ctx, butterband)]() {
      ctlbind(ctls, "grain_size [px]: ", CTL_CONTEXT(ctx, grain_size), "");
      ctlbind(ctls, "grain_band [px]: ", CTL_CONTEXT(ctx, grain_band), "");
      ctlbind(ctls, "order: ", CTL_CONTEXT(ctx, order), "");
      ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      ctlbind(ctls, "inverse: ", CTL_CONTEXT(ctx, inverse), "");
      });

  ctlbind_expandable_group(ctls, "Gaussian sharp options",
      [&, ctx = CTL_CONTEXT(ctx, gaussian_sharp)]() {
        ctlbind(ctls, "sigma [px]: ", CTL_CONTEXT(ctx, sigma), "Gaussian unsharp sigma in image space domain");
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });

  ctlbind_expandable_group(ctls, "Laplacian sharp options",
      [&, ctx = CTL_CONTEXT(ctx, laplacian_sharp )]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
        ctlbind(ctls, "bwrc: ", CTL_CONTEXT(ctx, bwrc), "Butterworth cutoff");
        ctlbind(ctls, "bworder: ", CTL_CONTEXT(ctx, bworder), "Butterworth order");
      });

  ctlbind_expandable_group(ctls, "Laplacian lpass options",
      [&, ctx = CTL_CONTEXT(ctx, laplacian_lpass )]() {
        ctlbind(ctls, "gain: ", CTL_CONTEXT(ctx, gain), "");
      });
}

bool c_fft_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _filterType);
    SERIALIZE_OPTION(settings, save, *this, _ppsDecomposition);
    SERIALIZE_OPTION(settings, save, *this, _swapQuadrants);

    if ( auto group = SERIALIZE_GROUP(settings, save, "GaussianFilter")) {
      SERIALIZE_OPTION(group, save, gaussian, sigma);
      SERIALIZE_OPTION(group, save, gaussian, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "LaplacianFilter")) {
      SERIALIZE_OPTION(group, save, laplacian, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "GradientFilter")) {
      SERIALIZE_OPTION(group, save, ramp, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "ButterworthFilter")) {
      SERIALIZE_OPTION(group, save, butterworth, rc);
      SERIALIZE_OPTION(group, save, butterworth, order);
      SERIALIZE_OPTION(group, save, butterworth, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "ButterBandFilter")) {
      SERIALIZE_OPTION(group, save, butterband, grain_size);
      SERIALIZE_OPTION(group, save, butterband, grain_band);
      SERIALIZE_OPTION(group, save, butterband, order);
      SERIALIZE_OPTION(group, save, butterband, gain);
      SERIALIZE_OPTION(group, save, butterband, inverse);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "GaussianSharpFilter")) {
      SERIALIZE_OPTION(group, save, gaussian_sharp, sigma);
      SERIALIZE_OPTION(group, save, gaussian_sharp, gain);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "LaplacianSharpFilter")) {
      SERIALIZE_OPTION(group, save, laplacian_sharp, gain);
      SERIALIZE_OPTION(group, save, laplacian_sharp, bwrc);
      SERIALIZE_OPTION(group, save, laplacian_sharp, bworder);
    }

    if ( auto group = SERIALIZE_GROUP(settings, save, "LaplacianLpassFilter")) {
      SERIALIZE_OPTION(group, save, laplacian_lpass, gain);
    }

    return true;
  }
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
      FILTER = fftGenerateGaussianFilter(fftSize, gaussian.sigma, gaussian.gain, false);
      break;
    }

    case FILTER_LAPLACIAN: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateLaplacianFilter(fftSize, laplacian.gain, false);
      break;
    }

    case FILTER_RAMP: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateRampFilter(fftSize, ramp.gain, false);
      break;
    }

    case FILTER_BUTTERWORTH: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateButterworthFilter(fftSize, butterworth.rc, butterworth.order, butterworth.gain, false);
      break;
    }

    case FILTER_BUTTERWORTH_BAND: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateButterworthBandFilter(fftSize, butterband.grain_size, butterband.grain_band,
          butterband.order, butterband.gain, butterband.inverse, false);
      break;
    }

    case FILTER_GAUSSIAN_SHARP: {
      const int ksize = std::max(3, std::min(63, 2 * int(3 * gaussian_sharp.sigma) + 1));
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateGaussianUnsharpFilter(fftSize, gaussian_sharp.sigma, gaussian_sharp.gain, false);
      break;
    }

    case FILTER_LAPLACIAN_SHARP: {
      const int ksize = 0;
      fftSize = fftGetOptimalSize(src.size(), cv::Size(ksize, ksize), &rc);
      FILTER = fftGenerateLaplacianUnsharpFilter(fftSize, laplacian_sharp.gain,
          laplacian_sharp.bwrc, laplacian_sharp.bworder, false);
      break;
    }

    default:
      CF_ERROR("Not supported filter=%d requested", _filterType);
      break;
  }

  if ( _display == DISPLAY_FILTER_MODULE ) {
    if ( _swapQuadrants ) {
      fftSwapQuadrants(FILTER, image);
    }
    else {
      image.assign(FILTER);
    }
    mask.release();
    return true;
  }

  if( !_ppsDecomposition ) {
    VLAP.release();
  }
  else if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
    CF_DEBUG("VLAP: %dx%d", VLAP.cols, VLAP.rows);
  }

  if ( _display == DISPLAY_VLAP ) {
    if ( _swapQuadrants ) {
      fftSwapQuadrants(VLAP, image);
    }
    else {
      VLAP.copyTo(image);
    }
    mask.release();
    return true;
  }

  std::vector<cv::Mat> real_channels(cn);
  std::vector<cv::Mat> complex_channels(cn);
  std::vector<cv::Mat> complex_channels_s(cn);
  std::vector<cv::Mat> complex_channels_v(cn);
  cv::split(src, real_channels);

  for ( int i = 0; i < cn; ++i ) {
    fftCopyMakeBorder(real_channels[i], real_channels[i], fftSize);
    real_channels[i].convertTo(real_channels[i], CV_32F);

    // if S must be also multiplied by the filter then use _ppsDecomposition = false
    if ( ! _ppsDecomposition ) {
      fftImageToSpectrum(real_channels[i], complex_channels[i], fftSize, false);
    }
    else {
      fftPPSDecomposition(real_channels[i], VLAP,
          complex_channels[i],
          complex_channels_s[i],
          complex_channels_v[i]);
    }
  }

  switch (_display) {
    case DISPLAY_SRC_SPECTRUM_MODULE:
    case DISPLAY_SRC_SPECTRUM_PHASE: {
      std::vector<cv::Mat> planes(cn);
      for( int i = 0; i < cn; ++i ) {
        if( _ppsDecomposition ) {
          cv::add(complex_channels[i], complex_channels_s[i], complex_channels[i]);
        }
        fftSpectrumToPolar(complex_channels[i], complex_channels[i]);
        cv::extractChannel(complex_channels[i], planes[i], _display == DISPLAY_SRC_SPECTRUM_MODULE ? 0 : 1);
      }
      if( cn == 1 ) {
        image.move(planes[0]);
      }
      else {
        cv::merge(planes, image);
      }
      if ( _swapQuadrants ) {
        fftSwapQuadrants(image);
      }
      mask.release();
      return true;
    }

    case DISPLAY_SRC_SPECTRUM_P_MODULE:
    case DISPLAY_SRC_SPECTRUM_P_PHASE: {
      if( !_ppsDecomposition ) {
        image.release();
      }
      else {
        std::vector<cv::Mat> planes(cn);
        for( int i = 0; i < cn; ++i ) {
          fftSpectrumToPolar(complex_channels[i], complex_channels[i]);
          cv::extractChannel(complex_channels[i], planes[i], _display == DISPLAY_SRC_SPECTRUM_P_MODULE ? 0 : 1);
        }
        if( cn == 1 ) {
          image.move(planes[0]);
        }
        else {
          cv::merge(planes, image);
        }
        if ( _swapQuadrants ) {
          fftSwapQuadrants(image);
        }
      }

      mask.release();
      return true;
    }

    case DISPLAY_SRC_SPECTRUM_S_MODULE:
    case DISPLAY_SRC_SPECTRUM_S_PHASE: {
      if( !_ppsDecomposition ) {
        image.release();
      }
      else {
        std::vector<cv::Mat> planes(cn);
        for( int i = 0; i < cn; ++i ) {
          fftSpectrumToPolar(complex_channels[i], complex_channels[i]);
          cv::extractChannel(complex_channels_s[i], planes[i], _display == DISPLAY_SRC_SPECTRUM_S_MODULE ? 0 : 1);
        }
        if( cn == 1 ) {
          image.move(planes[0]);
        }
        else {
          cv::merge(planes, image);
        }
        if ( _swapQuadrants ) {
          fftSwapQuadrants(image);
        }
      }
      mask.release();
      return true;
    }

    case DISPLAY_SRC_SPECTRUM_V_MODULE:
    case DISPLAY_SRC_SPECTRUM_V_PHASE: {
      if( !_ppsDecomposition ) {
        image.release();
      }
      else {
        std::vector<cv::Mat> planes(cn);
        for( int i = 0; i < cn; ++i ) {
          fftSpectrumToPolar(complex_channels[i], complex_channels[i]);
          cv::extractChannel(complex_channels_v[i], planes[i], _display == DISPLAY_SRC_SPECTRUM_V_MODULE ? 0 : 1);
        }
        if( cn == 1 ) {
          image.move(planes[0]);
        }
        else {
          cv::merge(planes, image);
        }
        if ( _swapQuadrants ) {
          fftSwapQuadrants(image);
        }
      }
      mask.release();
      return true;
    }

    default:
      break;
  }

  for ( int i = 0; i < cn; ++i ) {

    fftMulSpectrum(complex_channels[i], FILTER, complex_channels[i]);

    if ( _ppsDecomposition ) {
      // if S must be also multiplied by the filter then use _ppsDecomposition = false
      cv::add(complex_channels[i], complex_channels_s[i], complex_channels[i]);
    }

    if ( _display == DISPLAY_FILTERED_SPECTRUM_MODULE || _display == DISPLAY_FILTERED_SPECTRUM_PHASE ) {

      fftSpectrumToPolar(complex_channels[i], complex_channels[i]);

      cv::extractChannel(complex_channels[i], real_channels[i],
          _display == DISPLAY_FILTERED_SPECTRUM_MODULE ? 0 : 1);

      if ( _swapQuadrants ) {
        fftSwapQuadrants(real_channels[i]);
      }

      continue;
    }

    // DISPLAY_FILTERED_IMAGE
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

