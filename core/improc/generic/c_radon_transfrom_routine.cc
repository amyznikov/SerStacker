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
      { c_radon_transfrom_routine::DISPLAY_POLAR_MAGNITUDE, "POLAR_MAGNITUDE" },
      { c_radon_transfrom_routine::DISPLAY_POLAR_SPECTRUM, "POLAR_SPECTRUM" },
      { c_radon_transfrom_routine::DISPLAY_SINOGRAM, },
  };
  return members;
}

static void generatePolarSpectrumMaps(int max_len, int num_angles,
    float cx, float cy, double theta, double start_angle,
    cv::Mat1f & map_x, cv::Mat1f & map_y)
{
  INSTRUMENT_REGION("");

  map_x.create(num_angles, max_len);
  map_y.create(num_angles, max_len);

  const float angle_step = static_cast<float>(theta * M_PI / 180.0);
  const float start_rad = static_cast<float>(start_angle * M_PI / 180.0);

  const size_t map_x_stride = map_x.step / sizeof(float);
  const size_t map_y_stride = map_y.step / sizeof(float);
  float * const map_x_base = (float*)map_x.ptr();
  float * const map_y_base = (float*)map_y.ptr();

  parallel_for(0, num_angles, [=](const auto & range) {
    for ( int a = rbegin(range); a < rend(range); ++a ) {
      const float angle = start_rad + a * angle_step;
      const float ct = std::cos(angle);
      const float st = std::sin(angle);

      float * __restrict ptr_x = map_x_base + a * map_x_stride;
      float * __restrict ptr_y = map_y_base + a * map_y_stride;

      for ( int r_idx = 0; r_idx < max_len; ++r_idx ) {
        const float r = static_cast<float>(r_idx);
        ptr_x[r_idx] = cx + r * ct;
        ptr_y[r_idx] = cy + r * st;
      }
    }
  });
}

bool my_fft_radon_transform(cv::InputArray _src, cv::OutputArray _dst,
    double theta, double start_angle, double end_angle,
    cv::Mat & oututP_SPECTRUM,
    cv::Mat & oututPolarMagnitude,
    cv::Mat & oututPolarSpectrum)
{
  INSTRUMENT_REGION("");

  if ( _src.empty() || _src.type() != CV_32FC1 ) {
    CF_ERROR("my_fft_radon_transform: Single-channel CV_32FC1 input image expected");
    return false;
  }

  const cv::Mat src = _src.getMat();
  const cv::Size fftSize = src.size();

  const int rows = src.rows;
  const int cols = src.cols;

  const int num_angles = static_cast<int>(std::floor((end_angle - start_angle) / theta));
  if ( num_angles <= 0 ) {
    CF_ERROR("my_fft_radon_transform: Invalid angles range or theta step");
    return false;
  }

  const int max_len = static_cast<int>(std::ceil(std::sqrt(cols * cols + rows * rows)));

  cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
  if ( VLAP.empty() ) {
    CF_ERROR("my_fft_radon_transform: Failed to generate Discrete Laplacian Filter");
    return false;
  }

  cv::Mat P_SPECTRUM;
  if ( !fftPPSDecompositionCCS(src, VLAP, P_SPECTRUM, cv::noArray()) ) {
    CF_ERROR("my_fft_radon_transform: fftPPSDecompositionCCS failed");
    return false;
  }

  oututP_SPECTRUM = P_SPECTRUM;

  cv::Mat polarMagnitude;
  if ( !fftCCSSpectrumMagnitude(P_SPECTRUM, polarMagnitude, true) ) {
    CF_ERROR("my_fft_radon_transform: fftCCSSpectrumMagnitude failed");
    return false;
  }

  oututPolarMagnitude = polarMagnitude;

  cv::Mat1f map_x, map_y;
  const float cx = polarMagnitude.cols / 2.0f;
  const float cy = polarMagnitude.rows / 2.0f;
  generatePolarSpectrumMaps(max_len, num_angles, cx, cy, theta, start_angle, map_x, map_y);

  cv::Mat polar_spectrum;
  cv::remap(polarMagnitude, polar_spectrum, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  oututPolarSpectrum = polar_spectrum;

//  fftUnpackCCSSpectrum()

    cv::Mat dst_sinogram = createOutOfPlace(_src, _dst, num_angles, max_len, CV_32FC1);

    parallel_for(0, num_angles, [&](const auto & range) {
      for ( int a = rbegin(range); a < rend(range); ++a ) {
        cv::Mat src_row = polar_spectrum.row(a);
        cv::Mat dst_row = dst_sinogram.row(a);
        cv::dft(src_row, dst_row, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_ROWS);
      }
    });

    //dst_sinogram = dst_sinogram.t();

    assignOutOfPlace(_dst, dst_sinogram);
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
  ctlbind(ctls, "theta", ctx(&this_class::theta), "");
  ctlbind(ctls, "start_angle", ctx(&this_class::start_angle), "");
  ctlbind(ctls, "end_angle", ctx(&this_class::end_angle), "");
  ctlbind(ctls, "crop", ctx(&this_class::crop), "");
  ctlbind(ctls, "norm", ctx(&this_class::norm), "");
}

bool c_radon_transfrom_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat SINOGRAM;
  cv::Mat P_SPECTRUM;
  cv::Mat polarMagnitude;
  cv::Mat polarSpectrum;

  CF_DEBUG("Enter");
  my_fft_radon_transform(image, SINOGRAM, theta, start_angle, end_angle,
      P_SPECTRUM, polarMagnitude, polarSpectrum);
  CF_DEBUG("Leave");

  switch (_display) {
    case DISPLAY_P_SPECTRUM:
      image.move(P_SPECTRUM);
      break;
    case DISPLAY_POLAR_MAGNITUDE:
      image.move(polarMagnitude);
      break;
    case DISPLAY_POLAR_SPECTRUM:
      image.move(polarSpectrum);
      break;
    case DISPLAY_SINOGRAM:
    default:
      image.move(SINOGRAM);
      break;
  }

  mask.release();

  return true;
}

