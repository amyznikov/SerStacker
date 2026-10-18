/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>
#include <core/proc/divide.h>
#include <core/proc/multiply.h>


template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC1, "SRC1", "" },
      { c_alpha_test_routine::DISPLAY_SRC2, "SRC2", "" },
      { c_alpha_test_routine::DISPLAY_SPEC1, "SPEC1", "" },
      { c_alpha_test_routine::DISPLAY_SPEC2, "SPEC2", "" },
      { c_alpha_test_routine::DISPLAY_SPEC3, "SPEC3", "" },
      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13_DIFF, "SPEC13_DIFF", "" },
      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13, "CROSS_SPEC13", "" },
      { c_alpha_test_routine::DISPLAY_CORR_MAP13, "CORR_MAP13", "" },
      { c_alpha_test_routine::DISPLAY_SRC2}
  };
  return members;
}

/////////////////////////////////////
namespace {

void generateBandpassFilter1(cv::Mat1f & filter, const cv::Size & fftSize, double sigma)
{
  // Warning: for correct cross-correltion directly in CCS packed format the spectrum size must be even
  // otherwise the bandpass filter become not symmetrical due to embedded sign alternating
  // ilambda = _gsigma * (2 * sqrt(pi))
  // rho2 = (u^2 + v^2) * ilambda^2;
  // F(u, v) = rho2 * exp (-0.5 * rho2 )
  // the alternating +1 and -1 is also embedded into filter to avoid later fftSwapQudrants()

  INSTRUMENT_REGION("");

  filter.create(fftSize);

  const uint8_t * filter_base = filter.ptr();
  const size_t filter_stride = filter.step;

  const int cols = fftSize.width;
  const int rows = fftSize.height;

  if ( sigma <= 0 ) {
    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);
        const float start_sign = (y & 1) ? -1.0f : 1.0f;
        for( int x = 0; x < cols; ++x ) {
          fltp[x] = (x & 1) ? -start_sign : start_sign;
        }
      }
    });
  }
  else {
    const double ilambda_x = (2.0 * CV_PI * sigma) / cols;
    const double ilambda_y = (2.0 * CV_PI * sigma) / rows;
    const float ilambda_x2 = float(ilambda_x * ilambda_x);
    const float ilambda_y2 = float(ilambda_y * ilambda_y);

    parallel_for(0, rows, [=](const auto & range) {
      for( int y = rbegin(range); y < rend(range); ++y ) {
        float * __restrict fltp = (float * )(filter_base + y * filter_stride);

        const int fy = (y > rows / 2) ? (rows - y) : y;
        const float v2 = float(fy * fy * ilambda_y2);

        for( int x = 0; x < cols; ++x ) {
          const int fx = (x > cols / 2) ? (cols - x) : x;
          const int sign = ((x + y) & 1) ? -1 : 1;
          const float u2 = float(fx * fx * ilambda_x2);
          const float rho2 = u2 + v2;
          fltp[x] = sign * rho2 * rho2 * std::exp(-rho2);
          //fltp[x] = sign * rho2 * std::exp(-0.5f * rho2);
        }
      }
    });
  }

  cv::multiply(filter, 1. / cv::norm(filter, cv::NORM_L1), filter);
}

//bool fftRotateComplexSpec(const cv::Mat2f & src, cv::Mat2f & dst, double angleRad)
//{
//  const int rows = src.rows;
//  const int cols = src.cols;
//
//  dst.create(rows, cols);
//  dst.setTo(cv::Scalar::all(0));
//
//  cv::Mat1f cntr(rows, cols, 0.f);
//
//  const uint8_t * const src_base = src.ptr();
//  const size_t src_stride = src.step;
//
//  uint8_t * const dst_base = dst.ptr();
//  const size_t dst_stride = dst.step;
//
//  uint8_t * const cntr_base = cntr.ptr();
//  const size_t cntr_stride = cntr.step;
//
//  const float cx = cols / 2;
//  const float cy = rows / 2;
//
//  const float ca = std::cos(angleRad);
//  const float sa = std::sin(angleRad);
//
//  for (int y = 0; y < rows; ++y) {
//    const cv::Vec2f * const srcp = (const cv::Vec2f *)(src_base + y * src_stride);
//
//    const float dy = y - cy;
//
//    for (int x = 0; x < cols; ++x) {
//      const float dx = x - cx;
//      const float xx = cx + dx * ca - dy * sa;
//      const float yy = cy + dx * sa + dy * ca;
//
//      const int xc = int(xx);
//      const int yc = int(yy);
//
//      if ( yc >= 0 && yc < rows ) {
//        if ( xc >= 0 && xc < cols ) {
//          cv::Vec2f * __restrict dstp = (cv::Vec2f *)(dst_base + yc * dst_stride);
//          float * __restrict cntrp = (float *)(cntr_base + yc * cntr_stride);
//          dstp[xc] += srcp[x];
//          cntrp[xc] += 1;
//        }
//      }
//    }
//  }
//
//  divideImages(dst, cntr, dst);
//
//  return true;
//}

bool fftRotateComplexSpec(const cv::Mat2f & src, cv::Mat2f & dst, double angleRad)
{
  const int rows = src.rows;
  const int cols = src.cols;

  dst.create(rows, cols);
  dst.setTo(cv::Scalar::all(0));

  cv::Mat1f cntr(rows, cols, 0.f);

  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  const float cx = cols / 2;
  const float cy = rows / 2;

  const float ca = std::cos(angleRad);
  const float sa = std::sin(angleRad);

  for (int y = 0; y < rows; ++y) {
    const cv::Vec2f * const srcp = (const cv::Vec2f *)(src_base + y * src_stride);
    const float dy = y - cy;

    for (int x = 0; x < cols; ++x) {
      const float dx = x - cx;
      const float xx = cx + dx * ca - dy * sa;
      const float yy = cy + dx * sa + dy * ca;

      const int xc = int(xx);
      const int yc = int(yy);

      const float fx = xx - xc;
      const float fy = yy - yc;

      float wx[-1 + 2 + 1];
      float *wx_ptr = &wx[1];

      wx_ptr[-1] = std::max(0.0f, 0.5f - fx);
      wx_ptr[1]  = std::max(0.0f, fx - 0.5f);
      wx_ptr[0]  = 1.0f - wx_ptr[-1] - wx_ptr[1];

      float wy[-1 + 2 + 1];
      float *wy_ptr = &wy[1];

      wy_ptr[-1] = std::max(0.0f, 0.5f - fy);
      wy_ptr[1]  = std::max(0.0f, fy - 0.5f);
      wy_ptr[0]  = 1.0f - wy_ptr[-1] - wy_ptr[1];

      for (int k = -1; k <= 1; ++k) {
        const int target_y = yc + k;
        if (target_y < 0 || target_y >= rows) continue;

        const float w_y = wy_ptr[k];
        if (w_y <= 0.f) continue;

        cv::Vec2f * __restrict dstp = (cv::Vec2f *)(dst_base + target_y * dst_stride);
        float * __restrict cntrp = (float *)(cntr_base + target_y * cntr_stride);

        for (int m = -1; m <= 1; ++m) {
          const int target_x = xc + m;
          if (target_x < 0 || target_x >= cols) continue;

          const float w_x = wx_ptr[m];
          const float weight = w_x * w_y;

          if (weight > 0.f) {
            dstp[target_x] += srcp[x] * weight;
            cntrp[target_x] += weight;
          }
        }
      }
    }
  }

  divideImages(dst, cntr, dst);

  return true;
}

} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _translation);
    SERIALIZE_OPTION(settings, save, *this, _angle);
    SERIALIZE_OPTION(settings, save, *this, _sigma);
    SERIALIZE_OPTION(settings, save, *this, _whitenSpecs);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "translation", CTL_CONTEXT(ctx, _translation), "");
  ctlbind_slider_spinbox(ctls, "angle", CTL_CONTEXT(ctx, _angle), -180, 180, 0.5, "");
  ctlbind_slider_spinbox(ctls, "sigma", CTL_CONTEXT(ctx, _sigma), 0, 25, 0.5, "");
  ctlbind(ctls, "whitenSpecs", CTL_CONTEXT(ctx, _whitenSpecs), "");

}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1f img;
  if ( image.channels() == 1 ) {
    image.getMat().convertTo(img, CV_32F);
  }
  else {
    cv::cvtColor(image, img, cv::COLOR_BGR2GRAY);
    img.convertTo(img, CV_32F);
  }

  cv::subtract(img, cv::mean(img, mask), img);

  if ( true ) {
    // Check if apodization suppresses spectral rays: CONFIRMED
    // Keep rays visible for debug purposes.
    if ( APW.size() != img.size() ) {
      APW = fftCreateEllipticalApodizationWindow(img.size(), 100);
    }
    cv::multiply(img, APW, img);
  }


  cv::Mat src1, src2;
  cv::Rect validROI;
  const cv::Size fftSize = fftGetOptimalSize(img.size()) * 2;
  const cv::Vec2f OldCenter(fftSize.width / 2, fftSize.height / 2);
  const cv::Vec2f NewCenter = OldCenter + _translation;

  src1.create(fftSize, img.type());
  src1.setTo(0);
  img.copyTo(src1(cv::Rect(fftSize.width/4, fftSize.height/4, img.cols, img.rows)));

  const cv::Matx23f Transfrom =
      create_euclidean_transform(OldCenter, NewCenter,
          _angle * CV_PI / 180);

  cv::warpAffine(src1, src2, Transfrom, src1.size(),
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_REPLICATE);

  if ( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
  }

  cv::Mat P1, P2, P3;
  if ( !fftPPSDecompositionCCS(src1, VLAP, P1, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS(src1) fails");
    return false;
  }

  if ( !fftPPSDecompositionCCS(src2, VLAP, P2, cv::noArray()) ) {
    CF_ERROR("fftPPSDecompositionCCS(src2) fails");
    return false;
  }

  cv::Mat2f cmplxP1, cmplxP2, cmplxP3;
  if ( !fftUnpackCCSSpectrum(P1, cmplxP1, true) ) {
    CF_ERROR("fftUnpackCCSSpectrum(P1) fails");
    return false;
  }
  if ( !fftUnpackCCSSpectrum(P2, cmplxP2, true) ) {
    CF_ERROR("fftUnpackCCSSpectrum(P2) fails");
    return false;
  }

  fftRotateComplexSpec(cmplxP2, cmplxP3, _angle * CV_PI / 180);

  // Phase and Cross Correlations are implemented for CCS spectrums only
  fftSwapQuadrants(cmplxP3, P3);
  if ( !fftPackCCSSpectrum(P3, P3) ) {
    CF_ERROR("fftPackCCSSpectrum(cmplxP3) fails");
    return false;
  }

  cv::Mat specDiff;
  cv::Mat outputMap;
  cv::Mat crossSpec13;

  cv::subtract(cmplxP3, cmplxP1, specDiff);
  fftSpectrumToPolar(specDiff);

  if ( true ) {
    // Check if P3 is valid: some defects detected
    //fftSwapQuadrants(cmplxP3, cmplxP3);
    cv::idft(P3, outputMap, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  }
  else {
    // compute cross-correlation map, temporary disabled

    if ( BPF.size() != fftSize || _sigma != _oldsigma ) {
      _oldsigma = _sigma;
      generateBandpassFilter1(BPF, fftSize, _sigma);
    }

    if ( _whitenSpecs ) {
      if ( !fftPhaseCorrelateWeightedCCS(P1, P3, BPF, crossSpec13) ) {
        CF_ERROR("fftPhaseCorrelateWeightedCCS(P1, P3) fails");
        return false;

      }
    }
    else {
      if ( !fftCrossSpectrumWeightedCCS(P1, P3, BPF, crossSpec13) ) {
        CF_ERROR("fftCrossSpectrumWeightedCCS(P1, P3) fails");
        return false;
      }
    }

    cv::idft(crossSpec13, outputMap, cv::DFT_REAL_OUTPUT);
  }

  switch(_display)
  {
    case DISPLAY_SRC1:
      image.assign(src1);
      break;
    case DISPLAY_SRC2:
      image.assign(src2);
      break;
    case DISPLAY_SPEC1:
      fftSpectrumToPolar(cmplxP1, image);
      break;
    case DISPLAY_SPEC2:
      fftSpectrumToPolar(cmplxP2, image);
      break;
    case DISPLAY_SPEC3:
      fftSpectrumToPolar(cmplxP3, image);
      break;
    case DISPLAY_CROSS_SPEC13_DIFF:
      image.assign(specDiff);
      break;
    case DISPLAY_CROSS_SPEC13:
      fftSpectrumToPolarCCS(crossSpec13, image, true);
      break;
    case DISPLAY_CORR_MAP13:
      image.assign(outputMap);
      break;
  }

  mask.release();

  return true;
}


