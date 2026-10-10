/*
 * c_radon_transfrom_routine.cc
 *
 *  Created on: Oct 10, 2026
 *      Author: amyznikov
 */

#include "c_radon_transfrom_routine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/radon_transform.hpp>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/proc/fft.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<c_radon_transfrom_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_radon_transfrom_routine::DISPLAY_SINOGRAM, "SINOGRAM", },
      { c_radon_transfrom_routine::DISPLAY_P_SPECTRUM, "P_SPECTRUM" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_CART, "CMLPX_SPEC_CART" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_CART_POLAR, "CMLPX_SPEC_CART_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_POLAR, "CMLPX_SPEC_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_CMLPX_SPEC_POLAR_POLAR, "CMLPX_SPEC_POLAR_POLAR" },
      { c_radon_transfrom_routine::DISPLAY_SINOGRAM, },
  };
  return members;
}

/**
* @brief Generate transformation map for cv::remap (Cartesian -> Polar).
*   The radial dimension of the output matrix is ​​strictly tied to the physical FFT size to eliminate distortions.
*
* @param outputMap Output two-channel CV_32FC2 matrix for cv::remap.
* @param fftSize Size of the initial Cartesian spectrum (must be even).
* @param numAngles Number of discrete scanning angles (height of the output matrix).
* */
static void createCart2PolarRemap(cv::Mat2f & outputMap, const cv::Size & fftSize, const int numAngles)
{
  const float centerX = fftSize.width / 2;
  const float centerY = fftSize.height / 2;

  // The number of radial samples is strictly the distance from the center to the nearest frame edge.
  // Since fftSize is even, this is simply lossless integer division.
  const int numRadii = (fftSize.width < fftSize.height) ? (fftSize.width / 2) : (fftSize.height / 2);

  // The radial step equals 1 pixel of the source matrix.
  // The rIdx multiplier can be used directly.
  const float stepTheta = float(2.0 * CV_PI / numAngles);

  // The matrix is ​​created with the exact size physically justified by the FFT spectrum.
  outputMap.create(cv::Size(numRadii, numAngles));

  uint8_t * const outmap_base = outputMap.ptr();
  const size_t outmap_stride = outputMap.step;

  parallel_for(0, numAngles, [=](const auto & range) {
    for( int thetaIdx = rbegin(range); thetaIdx < rend(range); ++thetaIdx ) {
      cv::Vec2f * __restrict mp = (cv::Vec2f * ) (outmap_base + thetaIdx * outmap_stride);
      const float theta = float(thetaIdx) * stepTheta;
      const float cos_t = std::cos(theta);
      const float sin_t = std::sin(theta);

      // normalization factor for the square FFT frequency grid.
      // stretches the beam step along the diagonals (where max is 0.707, turning the step into 1.414)
      const float grid_scale = 1.0f / std::fmax(std::abs(cos_t), std::abs(sin_t));
      for( int rIdx = 0; rIdx < numRadii; ++rIdx ) {
        const float r = float(rIdx* grid_scale);
        mp[rIdx] = cv::Vec2f(centerX + r * cos_t, centerY + r * sin_t);
      }
    }
  });
}

bool fftRadonTransform(cv::InputArray _src, cv::OutputArray _dst,
    cv::Mat & outputDebugP_SPECTRUM,
    cv::Mat & outputDebugComplexSpecCart,
    cv::Mat & outputDebugComplexSpecPolar)
{
  INSTRUMENT_REGION("");

  if ( _src.empty() || _src.type() != CV_32FC1 ) {
    CF_ERROR("my_fft_radon_transform: Single-channel CV_32FC1 input image expected");
    return false;
  }

  const cv::Mat src = _src.getMat();
  const cv::Size fftSize = src.size();
  const int rows = fftSize.height;
  const int cols = fftSize.width;

  cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
  if ( VLAP.empty() ) {
    CF_ERROR("my_fft_radon_transform: Failed to generate Discrete Laplacian Filter");
    return false;
  }

  cv::Mat P_SPECTRUM, cmplxCart, cmplxPolar;
  cv::Mat2f polarRmap;

  if ( !fftPPSDecompositionCCS(src, VLAP, P_SPECTRUM, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS() failed");
    return false;
  }

  outputDebugP_SPECTRUM = P_SPECTRUM;
  if ( !fftUnpackCCSSpectrum(P_SPECTRUM, cmplxCart, true) ) {
    CF_ERROR("fftUnpackCCSSpectrum() failed");
    return false;
  }

  outputDebugComplexSpecCart = cmplxCart;
  createCart2PolarRemap(polarRmap, fftSize, 360);

  cv::remap(cmplxCart, cmplxPolar, polarRmap, cv::noArray(), cv::INTER_CUBIC,
      cv::BORDER_REPLICATE);

  outputDebugComplexSpecPolar = cmplxPolar;

  cv::dft(cmplxPolar, _dst, cv::DFT_INVERSE | cv::DFT_ROWS | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

  return true;
}


bool c_radon_transfrom_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, theta);
    SERIALIZE_OPTION(settings, save, *this, start_angle);
    SERIALIZE_OPTION(settings, save, *this, end_angle);
    SERIALIZE_OPTION(settings, save, *this, crop);
    SERIALIZE_OPTION(settings, save, *this, norm);
    SERIALIZE_OPTION(settings, save, *this, _display);
    return true;
  }
  return false;
}

void c_radon_transfrom_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display), "");
//  ctlbind(ctls, "theta", ctx(&this_class::theta), "");
//  ctlbind(ctls, "start_angle", ctx(&this_class::start_angle), "");
//  ctlbind(ctls, "end_angle", ctx(&this_class::end_angle), "");
//  ctlbind(ctls, "crop", ctx(&this_class::crop), "");
//  ctlbind(ctls, "norm", ctx(&this_class::norm), "");
}

bool c_radon_transfrom_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat SINOGRAM;
  cv::Mat P_SPECTRUM;
  cv::Mat cmplxSpecCart;
  cv::Mat cmplxSpecPolar;

  CF_DEBUG("Enter");
  fftRadonTransform(image, SINOGRAM, P_SPECTRUM, cmplxSpecCart, cmplxSpecPolar);
  CF_DEBUG("Leave");

  switch (_display) {
    case DISPLAY_P_SPECTRUM:
      image.move(P_SPECTRUM);
      break;
    case DISPLAY_CMLPX_SPEC_CART:
      image.move(cmplxSpecCart);
      break;
    case DISPLAY_CMLPX_SPEC_CART_POLAR:
      image.move(cmplxSpecCart);
      fftSpectrumToPolar(image);
      break;
    case DISPLAY_CMLPX_SPEC_POLAR:
      image.move(cmplxSpecPolar);
      break;
    case DISPLAY_CMLPX_SPEC_POLAR_POLAR:
    image.move(cmplxSpecPolar);
    fftSpectrumToPolar(image);
    break;
    case DISPLAY_SINOGRAM:
    default:
      image.move(SINOGRAM);
      break;
  }

  mask.release();

  return true;
}

