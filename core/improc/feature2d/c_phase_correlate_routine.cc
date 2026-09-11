/*
 * c_phase_correlate_routine.cc
 *
 *  Created on: Sep 8, 2026
 *      Author: amyznikov
 */

#include "c_phase_correlate_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/run-loop.h>
#include <core/proc/fft.h>

template<>
const c_enum_member * members_of<c_phase_correlate_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_phase_correlate_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_BLEND_IMAGE, "BLEND_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_SHIFTED_CURRENT_IMAGE, "SHIFTED_CURRENT_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_SHIFTED_BLEND_IMAGE, "SHIFTED_BLEND_IMAGE", "" },

      { c_phase_correlate_routine::DISPLAY_CURRENT_SCALED_IMAGE,"CURRENT_SCALED_IMAGE"},
      { c_phase_correlate_routine::DISPLAY_REFERENCE_SCALED_IMAGE,"REFERENCE_SCALED_IMAGE"},
      { c_phase_correlate_routine::DISPLAY_CORRELATION_MAP, "CORRELATION_MAP", "" },

      { c_phase_correlate_routine::DISPLAY_CROSS_SPECTRUM_CART, "CROSS_SPECTRUM_CART", "" },
      { c_phase_correlate_routine::DISPLAY_CROSS_SPECTRUM_POLAR, "CROSS_SPECTRUM_POLAR", "" },

      { c_phase_correlate_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {

//static void unpackCrossSpectrumCCS(const cv::Mat1f & crossSpectrumCCS,
//    cv::Mat2f & complexSpectrum)
//{
//  const int rows = crossSpectrumCCS.rows;
//  const int cols = crossSpectrumCCS.cols;
//  const bool is_even_cols = (cols % 2 == 0);
//  const bool is_even_rows = (rows % 2 == 0);
//
//  complexSpectrum.create(rows, cols);
//  complexSpectrum.setTo(0);
//
//  const uint8_t * ccs_base = crossSpectrumCCS.ptr();
//  const size_t ccs_stride = crossSpectrumCCS.step;
//
//  uint8_t * dst_base = complexSpectrum.ptr();
//  const size_t dst_stride = complexSpectrum.step;
//
//  parallel_for(0, rows, [=](const auto & range) {
//    for (int y = rbegin(range); y < rend(range); ++y) {
//
//      const int mirror_y = (y == 0) ? 0 : (rows - y);
//      const float* srcp = (const float*)(ccs_base + y * ccs_stride);
//      float* dstp1 = (float*)(dst_base + y * dst_stride);
//      float* dstp2 = (float*)(dst_base + mirror_y * dst_stride);
//      const float sign_y = (y % 2 == 0) ? 1.0f : -1.0f;
//
//      // DC (x = 0, fx = 0)
//      const float dc_re = srcp[0] * sign_y;
//      dstp1[0] = dc_re; dstp1[1] = 0.0f;
//      dstp2[0] = dc_re; dstp2[1] = 0.0f;
//
//      const int max_complex_idx = is_even_cols ? (cols - 2) : (cols - 1);
//      for (int x = 1; x <= max_complex_idx; x += 2) {
//        const int fx = (x + 1) / 2;
//        const float sign_fx = ((fx + y) % 2 == 0) ? 1.0f : -1.0f;
//
//        const float re = srcp[x] * sign_fx;
//        const float im = srcp[x + 1] * sign_fx;
//        const int mirror_x = cols - fx;
//
//        dstp1[fx * 2] = re;
//        dstp1[fx * 2 + 1] = im;
//
//        dstp2[mirror_x * 2] = re;
//        dstp2[mirror_x * 2 + 1] = -im;
//      }
//
//      if (is_even_cols) {
//        const int fx_nyquist = cols / 2;
//        const float sign_nyquist = ((fx_nyquist + y) % 2 == 0) ? 1.0f : -1.0f;
//        const float nyq_re = srcp[cols - 1] * sign_nyquist;
//        dstp1[fx_nyquist * 2] = nyq_re; dstp1[fx_nyquist * 2 + 1] = 0.0f;
//        dstp2[fx_nyquist * 2] = nyq_re; dstp2[fx_nyquist * 2 + 1] = 0.0f;
//      }
//    }
//  });
//}

static void unpackCrossSpectrumCCS(const cv::Mat1f & crossSpectrumCCS,
    cv::OutputArray _complexSpectrum)
{
  const int rows = crossSpectrumCCS.rows;
  const int cols = crossSpectrumCCS.cols;
  const bool is_even_cols = (cols % 2 == 0);
  const bool is_even_rows = (rows % 2 == 0);

  _complexSpectrum.create(rows, cols, CV_32FC2);
  cv::Mat2f complexSpectrum = _complexSpectrum.getMatRef();

  const uint8_t * ccs_base = crossSpectrumCCS.ptr();
  const size_t ccs_stride = crossSpectrumCCS.step;

  uint8_t * dst_base = complexSpectrum.ptr();
  const size_t dst_stride = complexSpectrum.step;

  parallel_for(0, rows, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {

      const int mirror_y = (y == 0) ? 0 : (rows - y);
      const float* srcp = (const float*)(ccs_base + y * ccs_stride);
      float* dstp1 = (float*)(dst_base + y * dst_stride);
      float* dstp2 = (float*)(dst_base + mirror_y * dst_stride);
      const float sign_y = (y % 2 == 0) ? 1.0f : -1.0f;

      // DC (x = 0, fx = 0)
      const float dc_re = srcp[0] * sign_y;
      dstp1[0] = dc_re; dstp1[1] = 0.0f;
      dstp2[0] = dc_re; dstp2[1] = 0.0f;

      const int max_complex_idx = is_even_cols ? (cols - 2) : (cols - 1);
      for (int x = 1; x <= max_complex_idx; x += 2) {
        const int fx = (x + 1) / 2;
        const float sign_fx = ((fx + y) % 2 == 0) ? 1.0f : -1.0f;

        const float re = srcp[x] * sign_fx;
        const float im = srcp[x + 1] * sign_fx;
        const int mirror_x = cols - fx;

        dstp1[fx * 2] = re;
        dstp1[fx * 2 + 1] = im;

        dstp2[mirror_x * 2] = re;
        dstp2[mirror_x * 2 + 1] = -im;
      }

      if (is_even_cols) {
        const int fx_nyquist = cols / 2;
        const float sign_nyquist = ((fx_nyquist + y) % 2 == 0) ? 1.0f : -1.0f;
        const float nyq_re = srcp[cols - 1] * sign_nyquist;
        dstp1[fx_nyquist * 2] = nyq_re; dstp1[fx_nyquist * 2 + 1] = 0.0f;
        dstp2[fx_nyquist * 2] = nyq_re; dstp2[fx_nyquist * 2 + 1] = 0.0f;
      }
    }
  });
}

} // namespace

bool c_phase_correlate_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _fillMaskHoles);
    serialize_phase_correlate_options(settings, save, opts);
    return true;
  }
  return false;
}

void c_phase_correlate_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "downscaleFactor", ctx,  &this_class::downscaleFactor, &this_class::set_downscaleFactor, "");
  ctlbind(ctls, "gsigma", ctx,  &this_class::gsigma, &this_class::set_gsigma, "");
  ctlbind(ctls, "apodizationSize", ctx,  &this_class::apodizationSize, &this_class::set_apodizationSize, "");
  ctlbind(ctls, "fillMaskHoles", CTL_CONTEXT(ctx, _fillMaskHoles), "Set checked to call geo_fill_holes(currentMask)");
  ctlbind(ctls, "updateReference", CTL_CONTEXT(ctx, _updateReferenceImage), "Set checked to set current image as reference");
  ctlbind(ctls, "printScores", CTL_CONTEXT(ctx, _printScores), "Set checked to dump debug info");

}

bool c_phase_correlate_routine::reinitialize(const cv::Size & expectedFrameSize)
{
  return (_initialized = pc.setup(expectedFrameSize, opts));
}

bool c_phase_correlate_routine::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  if ( currentImage.empty() ) {
    CF_ERROR("currentImage is empty");
    return false;
  }

  if (!currentMask.empty() && currentMask.channels() != 1 ) {
    CF_ERROR("currentMask must be single-channel");
    return false;
  }

  if( !_initialized && !reinitialize(currentImage.size())) {
    CF_ERROR("ERROR: reinitialize() fails");
    return false;
  }

  if ( currentImage.channels() == 1 ) {
    currentImage.getMat().convertTo(_currentImage, CV_32F);
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(currentImage, tmp, cv::COLOR_BGR2GRAY);
    tmp.convertTo(_currentImage, CV_32F);
  }

  currentMask.copyTo(_currentMask);

  return pc.setCurrentImage(_currentImage, _currentMask);
}

bool c_phase_correlate_routine::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if ( referenceImage.empty() ) {
    CF_ERROR("referenceImage is empty");
    return false;
  }

  if (!referenceMask.empty() && referenceMask.channels() != 1 ) {
    CF_ERROR("referenceMask must be single-channel");
    return false;
  }

  if( !_initialized && !reinitialize(referenceImage.size())) {
    CF_ERROR("ERROR: reinitialize() fails");
    return false;
  }

  if ( referenceImage.channels() == 1 ) {
    referenceImage.getMat().convertTo(_referenceImage, CV_32F);
  }
  else {
    cv::Mat tmp;
    cv::cvtColor(referenceImage, tmp, cv::COLOR_BGR2GRAY);
    tmp.convertTo(_referenceImage, CV_32F);
  }

  referenceMask.copyTo(_referenceMask);

  return pc.setReferenceImage(_referenceImage, _referenceMask);
}

static void shiftImage(cv::InputArray src, cv::OutputArray dst, const cv::Vec2f& translation)
{
    float data[6] = {
        1.0f, 0.0f, translation[0],
        0.0f, 1.0f, translation[1]
    };
    cv::Mat M(2, 3, CV_32FC1, data);
    cv::warpAffine(src, dst, M, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

bool c_phase_correlate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( (!_initialized || _updateReferenceImage) && !reinitialize(image.size()) )  {
    CF_ERROR("reinitialize() fails");
    return false;
  }

  if ( _referenceImage.empty() || _updateReferenceImage ) {
    if ( !setReferenceImage(image, mask) ) {
      CF_ERROR("setReferenceImage() fails");
      return false;
    }
  }

  if ( !_referenceImage.empty() ) {
    if ( !setCurrentImage(image, mask) ) {
      CF_ERROR("setCurrentImage() fails");
      return false;
    }

    cv::Vec2f Translation;
    double score;

    //CF_DEBUG("Call compute()");
    pc.setCurrentImage(_currentImage, _currentMask);
    score = pc.compute(Translation);
    if ( _printScores ) {
      CF_DEBUG("score: %g T: x=%g y=%g", score, Translation[0], Translation[1]);
    }

    switch (_display)
    {
      case DISPLAY_CURRENT_IMAGE:
        _currentImage.copyTo(image);
        _currentMask.copyTo(mask);
        break;
      case DISPLAY_REFERENCE_IMAGE:
        _referenceImage.copyTo(image);
        _referenceMask.copyTo(mask);
        break;
      case DISPLAY_SHIFTED_CURRENT_IMAGE : {
        shiftImage(_currentImage, image, Translation);
        mask.release();
        break;
      }
      case DISPLAY_BLEND_IMAGE: {
        if ( !_currentImage.empty() && !_referenceImage.empty() ) {
          cv::addWeighted(_currentImage, 0.5, _referenceImage, 0.5, 0, image);
        }
        else if ( !_currentImage.empty() ) {
          _currentImage.copyTo(image);
        }
        else if ( !_referenceImage.empty() ) {
          _referenceImage.copyTo(image);
        }
        else {
          // do nothing, keep ioutput image as is
        }
        mask.release();
        break;
      }
      case DISPLAY_SHIFTED_BLEND_IMAGE: {
        if ( !_currentImage.empty() && !_referenceImage.empty() ) {
          cv::Mat tmp;
          shiftImage(_currentImage, tmp, Translation);
          cv::addWeighted(tmp, 0.5, _referenceImage, 0.5, 0, image);
        }
        else if ( !_currentImage.empty() ) {
          cv::Mat tmp;
          shiftImage(_currentImage, tmp, Translation);
          tmp.copyTo(image);
        }
        else if ( !_referenceImage.empty() ) {
          _referenceImage.copyTo(image);
        }
        else {
          // do nothing, keep ioutput image as is
        }

        mask.release();
        break;
      }
      case DISPLAY_CURRENT_SCALED_IMAGE: {
        pc.scaledCurrentImage().copyTo(image);
        pc.scaledCurrentMask().copyTo(mask);
        break;
      }
      case DISPLAY_REFERENCE_SCALED_IMAGE: {
        pc.scaledReferenceImage().copyTo(image);
        pc.scaledReferenceMask().copyTo(mask);
        break;
      }
      case DISPLAY_CORRELATION_MAP: {
        pc.correlationMap().copyTo(image);
        mask.release();
        break;
      }
      case DISPLAY_CROSS_SPECTRUM_CART: {
        unpackCrossSpectrumCCS(pc.crossSpectrum(), image);
        mask.release();
        break;
      }
      case DISPLAY_CROSS_SPECTRUM_POLAR: {
        unpackCrossSpectrumCCS(pc.crossSpectrum(), image);
        fftSpectrumToPolar(image, image);
        mask.release();
        break;
      }
    }

  }
  return true;
}
