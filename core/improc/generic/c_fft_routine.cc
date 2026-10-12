/*
 * c_fft_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_fft_routine.h"
#include <core/proc/run-loop.h>
#include <core/ssprintf.h>


namespace {

/**
* @brief Generate transformation map for cv::remap (Cartesian -> Polar).
*   The radial dimension of the output matrix is ​​strictly tied to the physical FFT size to eliminate distortions.
*
* @param outputMap Output two-channel CV_32FC2 matrix for cv::remap.
* @param fftSize Size of the initial Cartesian spectrum (must be even).
* @param numAngles Number of discrete scanning angles (height of the output matrix).
* */
//static void fftCreateCart2PolarRemap(cv::Mat2f & outputMap, const cv::Size & fftSize, const int numAngles)
//{
//  const float centerX = fftSize.width / 2;
//  const float centerY = fftSize.height / 2;
//
//  // The number of radial samples is strictly the distance from the center to the nearest frame edge.
//  // Since fftSize is even, this is simply lossless integer division.
//  const int numRadii = (fftSize.width < fftSize.height) ? (fftSize.width / 2) : (fftSize.height / 2);
//
//  // The radial step equals 1 pixel of the source matrix.
//  // The rIdx multiplier can be used directly.
//  const float stepTheta = float(2.0 * CV_PI / numAngles);
//
//  // The matrix is ​​created with the exact size physically justified by the FFT spectrum.
//  outputMap.create(cv::Size(numRadii, numAngles));
//
//  uint8_t * const outmap_base = outputMap.ptr();
//  const size_t outmap_stride = outputMap.step;
//
//  parallel_for(0, numAngles, [=](const auto & range) {
//    for( int thetaIdx = rbegin(range); thetaIdx < rend(range); ++thetaIdx ) {
//      cv::Vec2f * __restrict mp = (cv::Vec2f * ) (outmap_base + thetaIdx * outmap_stride);
//      const float theta = float(thetaIdx) * stepTheta;
//      const float cos_t = std::cos(theta);
//      const float sin_t = std::sin(theta);
//
//      // normalization factor for the square FFT frequency grid.
//      // stretches the beam step along the diagonals (where max is 0.707, turning the step into 1.414)
//      const float grid_scale = 1.0f / std::fmax(std::abs(cos_t), std::abs(sin_t));
//      for( int rIdx = 0; rIdx < numRadii; ++rIdx ) {
//        const float r = float(rIdx* grid_scale);
//        mp[rIdx] = cv::Vec2f(centerX + r * cos_t, centerY + r * sin_t);
//      }
//    }
//  });
//}

/**
* @brief Generate transformation map for cv::remap (Cartesian -> Log-Polar).
*   The radial dimension of the output matrix is strictly logarithmic to eliminate distortions in Fourier-Mellin transform.
*
* @param outputMap Output two-channel CV_32FC2 matrix for cv::remap.
* @param fftSize Size of the initial Cartesian spectrum (must be even).
* @param numAngles Number of discrete scanning angles (height of the output matrix).
* */
/**
* @brief Generate transformation map for cv::remap (Cartesian -> Log-Polar).
*   The radial dimension of the output matrix is strictly logarithmic to eliminate distortions in Fourier-Mellin transform.
*
* @param outputMap Output two-channel CV_32FC2 matrix for cv::remap.
* @param fftSize Size of the initial Cartesian spectrum (must be even).
* @param numAngles Number of discrete scanning angles (height of the output matrix).
* */
static void fftCreateCart2PolarRemap(cv::Mat2f & outputMap, const cv::Size & fftSize, const int numAngles)
{
  const float centerX = fftSize.width / 2.0f;
  const float centerY = fftSize.height / 2.0f;

  // The number of radial samples is strictly the distance from the center to the nearest frame edge.
  // Since fftSize is even, this is simply lossless integer division.
  const int numRadii = (fftSize.width < fftSize.height) ? (fftSize.width / 2) : (fftSize.height / 2);

  // Minimum radius to avoid log(0) singularity. 1.0 pixel is standard for FFT spectrum.
  const float rMin = 1.0f;
  const float maxRadiusBase = float(numRadii);

  // Global uniform logarithmic step to guarantee rotation invariance across all angles.
  const float logStep = std::log(maxRadiusBase / rMin) / float(numRadii - 1);
  const float stepTheta = float(2.0 * CV_PI / numAngles);

  // The matrix is created with the exact size physically justified by the FFT spectrum.
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
      const float grid_scale = 1.0f / std::fmax(std::abs(cos_t), std::abs(sin_t));
      const float maxAllowedRadius = maxRadiusBase * grid_scale;

      for( int rIdx = 0; rIdx < numRadii; ++rIdx ) {
        // True exponential radius vector for Log-Polar scaling
        const float r = rMin * std::exp(float(rIdx) * logStep);

        // Safeguard for the square FFT matrix corners
        if (r > maxAllowedRadius) {
          mp[rIdx] = cv::Vec2f(-1.0f, -1.0f);
        }
        else {
          mp[rIdx] = cv::Vec2f(centerX + r * cos_t, centerY + r * sin_t);
        }
      }
    }
  });
}


}
template<>
const c_enum_member * members_of<c_fft_routine::DisplayType>()
{

  static const c_enum_member members[] = {
      {c_fft_routine::DisplayMagnitude, "Magnitude", "Display spectrum magnitude"},
      {c_fft_routine::DisplayPower, "Power", "Display power spectrum"},
      {c_fft_routine::DisplayPhase, "Phase", "Display phase spectrum"},
      {c_fft_routine::DisplayReal, "Real", "Display Real part of spectrum"},
      {c_fft_routine::DisplayImag, "Imag", "Display Imaginary part of spectrum"},
      {c_fft_routine::DisplayPolar, "Polar", "Display Polar"},
      {c_fft_routine::DisplayMagnitudePolar, "Magnitude in Polar coordinates", ""},
//      {c_fft_routine::DisplayCCSTest3, "CCSTest3", "CCSTest3"},
//      {c_fft_routine::DisplayCCSTest4, "CCSTest4", "CCSTest4"},
//      {c_fft_routine::DisplayCCSTest5, "CCSTest5", "CCSTest5"},
//      {c_fft_routine::DisplayCCSTest6, "CCSTest6", "CCSTest6"},
      {c_fft_routine::DisplayMagnitude},
  };

  return members;
}

void c_fft_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "Display", ctx(&this_class::_outputDisplay), "Output image display");
   ctlbind(ctls, "ppsDecomposition", ctx(&this_class::_ppsDecomposition), "Use pps decomposition");
   ctlbind(ctls, "dftComplex", ctx(&this_class::_dftComplex), "Set cv::DFT_COMPLEX_OUTPUT flag");
   ctlbind(ctls, "dftScale", ctx(&this_class::_dftScale), "Set cv::DFT_SCALE flag");
   ctlbind(ctls, "centerDC", ctx(&this_class::_centerDC), "");

}

bool c_fft_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _outputDisplay);
    SERIALIZE_OPTION(settings, save, *this, _ppsDecomposition);
    SERIALIZE_OPTION(settings, save, *this, _dftScale);
    SERIALIZE_OPTION(settings, save, *this, _dftComplex);
    SERIALIZE_OPTION(settings, save, *this, _centerDC);
    return true;
  }
  return false;
}

bool c_fft_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int cn = image.channels();

  std::vector<cv::Mat> channels(cn);

  if( cn == 1 ) {
    image.getMat().convertTo(channels[0], CV_32F);
  }
  else {
    cv::split(image.getMat(), channels);
    for( int c = 0; c < cn; ++c ) {
      channels[c].convertTo(channels[c], CV_32F);
    }
  }

  int flags = _dftComplex ? cv::DFT_COMPLEX_OUTPUT : cv::DFT_REAL_OUTPUT;
  if( _dftScale ) {
    flags |= cv::DFT_SCALE;
  }

  for( int c = 0; c < cn; ++c ) {
    cv::dft(channels[c], channels[c], flags);
  }

  switch (_outputDisplay) {
    case DisplayMagnitude:
      for( int c = 0; c < cn; ++c ) {
        if ( channels[c].channels() == 2 ) {
          fftSpectrumMagnitude(channels[c], channels[c], _centerDC);
        }
        else {
          fftCCSSpectrumMagnitude(channels[c], channels[c], _centerDC);
        }
      }
      break;

    case DisplayPower:
      for( int c = 0; c < cn; ++c ) {
        if ( channels[c].channels() == 2 ) {
          fftSpectrumPower(channels[c], channels[c], _centerDC);
        }
        else {
          fftCCSSpectrumPower(channels[c], channels[c], _centerDC);
        }
      }
      break;


    case DisplayPolar:
      for( int c = 0; c < cn; ++c ) {
        if ( channels[c].channels() == 1 ) {
          fftCCSSpectrumToPolar(channels[c], channels[c], _centerDC);
        }
        else {
          fftSpectrumToPolar(channels[c], channels[c]);
          if ( _centerDC ) {
            fftSwapQuadrants(channels[c], channels[c]);
          }
        }
      }
      break;

    case DisplayPhase:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumPhase(channels[c], channels[c]);
      }
      break;

    case DisplayReal:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 0);
      }
      break;

    case DisplayImag:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 1);
      }
      break;

    case DisplayMagnitudePolar:
      for( int c = 0; c < cn; ++c ) {
        if ( channels[c].channels() == 2 ) {
          fftSpectrumMagnitude(channels[c], channels[c], true);
        }
        else {
          fftCCSSpectrumMagnitude(channels[c], channels[c], true);
        }

        if ( polarReamp.size() != channels[c].size() ) {
          fftCreateCart2PolarRemap(polarReamp, channels[c].size(), 512);
        }

        cv::remap(channels[c], channels[c], polarReamp, cv::noArray(),
            cv::INTER_LINEAR, cv::BORDER_REPLICATE);
      }
      break;
  }

  if ( cn == 1 ) {
    channels[0].copyTo(image);
  }
  else {
    cv::merge(channels, image);
  }

  mask.release();

  return true;
}

