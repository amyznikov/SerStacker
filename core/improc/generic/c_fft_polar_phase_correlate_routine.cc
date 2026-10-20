/*
 * c_fft_polar_phase_correlate_routine.cc
 *
 *  Created on: Oct 16, 2026
 *      Author: amyznikov
 *
 *
 * «An FFT-based technique for translation, rotation, and scale-invariant image registration»
 *   (B. S. Reddy, B. N. Chatterji, 1996).
 *
 * «Extension of Phase Correlation to Rotation, Scale and Translation Invariant Image Registration»
 * (Emmanuel Sarvaiya, S. Patnaik).
 *
 * «Image Registration Using Log Polar Transform and Phase Correlation to Recover Higher Scale»
 * (Journal of Pattern Recognition Research, 2012).
 */

#include "c_fft_polar_phase_correlate_routine.h"
#include <core/proc/image_registration/image_transform.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_fft_polar_phase_correlate_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", },
      { c_fft_polar_phase_correlate_routine::DISPLAY_rotatedCurrentImage, "rotatedCurrentImage" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_alignedImage, "alignedImage" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_SPECTRUM, "REFERENCE_SPECTRUM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_SPECTRUM, "CURRENT_SPECTRUM", },
      { c_fft_polar_phase_correlate_routine::DISPLAY_rotatedCurrentSpec, "rotatedCurrentSpec" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM, "CURRENT_ACM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM, "REFERENCE_ACM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM_POLAR, "CURRENT_ACM_POLAR" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM_POLAR, "REFERENCE_ACM_POLAR" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM_POLAR_SPEC, "CURRENT_ACM_POLAR_SPEC" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_REFERENCE_ACM_POLAR_SPEC, "REFERENCE_ACM_POLAR_SPEC" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_POLAR_CROSS_SPECTRUM, "POLAR_CROSS_SPECTRUM" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_POLAR_CORRELATION_MAP, "POLAR_CORRELATION_MAP" },

      { c_fft_polar_phase_correlate_routine::DISPLAY_crossSpectrum, "crossSpectrum" },
      { c_fft_polar_phase_correlate_routine::DISPLAY_correlationMap, "correlationMap" },



      { c_fft_polar_phase_correlate_routine::DISPLAY_CURRENT_ACM, },
  };
  return members;
}

bool c_fft_polar_phase_correlate_routine::reinitialize(const cv::Size & expectedFrameSize)
{
  return (_initialized = pc.setup(expectedFrameSize, opts));
}

bool c_fft_polar_phase_correlate_routine::setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask)
{
  INSTRUMENT_REGION("");
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

bool c_fft_polar_phase_correlate_routine::setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  INSTRUMENT_REGION("");
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


bool c_fft_polar_phase_correlate_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    if ( auto group = SERIALIZE_GROUP(settings, save, "opts")) {
      serialize_polar_phase_correlate_options(group, save, opts);
    }
    return true;
  }
  return false;
}

void c_fft_polar_phase_correlate_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display), "display");
  ctlbind(ctls, "gsigma1", ctx,  &this_class::gsigma, &this_class::set_gsigma, "");
  ctlbind(ctls, "gsigma2", ctx,  &this_class::gsigma2, &this_class::set_gsigma2, "");
  ctlbind(ctls, "apodization", ctx,  &this_class::apodization, &this_class::set_apodization, "");
  ctlbind(ctls, "whitenPolarSpecs", ctx,  &this_class::whitenPolarSpecs, &this_class::set_whitenPolarSpecs, "");
  ctlbind(ctls, "whitenTransSpecs", ctx,  &this_class::whitenTransSpecs, &this_class::set_whitenTransSpecs, "");
  ctlbind(ctls, "maxPooling", ctx,  &this_class::maxPooling, &this_class::set_maxPooling, "");
  ctlbind(ctls, "estimateRotation", ctx,  &this_class::estimateRotation, &this_class::set_estimateRotation, "");
  ctlbind(ctls, "estimateTranslation", ctx,  &this_class::estimateTranslation, &this_class::set_estimateTranslation, "");
  ctlbind(ctls, "updateReference", ctx(&this_class::_updateReference), "updateReference");
}

static void remapImage(cv::InputArray src, cv::OutputArray dst, const cv::Size & dstSize,
    double rotationRadians, const cv::Vec2f & estimatedTranslation)
{
  if (src.empty()) {
    return;
  }

  const cv::Vec2f C_ref(dstSize.width / 2.f, dstSize.height / 2.f);
  const cv::Vec2f T_src(src.cols() / 2.f, src.rows() / 2.f);
  const cv::Vec2f C_mosaic = C_ref - estimatedTranslation;

  c_euclidean_image_transform transform(C_mosaic, T_src, -rotationRadians, 1.0f);
  ((c_image_transform&)transform).remap(src, cv::noArray(), dstSize, dst, cv::noArray());
}

static void warpImage(cv::InputArray src, cv::OutputArray dst, const cv::Size & dstSize,
    double rotationRadians, const cv::Vec2f & estimatedTranslation)
{
  if (src.empty()) {
    return;
  }

  const cv::Vec2f C_ref(dstSize.width / 2.f, dstSize.height / 2.f);
  const cv::Vec2f T_src(src.cols() / 2.f, src.rows() / 2.f);
  const cv::Vec2f C_mosaic = C_ref - estimatedTranslation;

  const double alpha = -rotationRadians;
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  // (dst -> src)
  cv::Matx23d M;
  M(0, 0) = ca;
  M(0, 1) = -sa;
  M(0, 2) = T_src[0] - ca * C_mosaic[0] + sa * C_mosaic[1];
  M(1, 0) = sa;
  M(1, 1) = ca;
  M(1, 2) = T_src[1] - sa * C_mosaic[0] - ca * C_mosaic[1];

  cv::warpAffine(src, dst, M, dstSize,
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, cv::Scalar::all(0));
}

bool c_fft_polar_phase_correlate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _referenceImage.empty() || _updateReference ) {
    if ( !reinitialize(image.size())) {
      CF_ERROR("reinitialize() fails");
      return false;
    }
    if ( !setReferenceImage(image, mask) ) {
      CF_ERROR("setReferenceImage() fails");
      return false;
    }
  }
  else if ( !_initialized ) {
    if ( !reinitialize(_referenceImage.size())) {
      CF_ERROR("reinitialize() fails");
      return false;
    }
    if ( !setReferenceImage(_referenceImage, _referenceMask) ) {
      CF_ERROR("setReferenceImage() fails");
      return false;
    }
  }

  if ( !setCurrentImage(image, mask) ) {
    CF_ERROR("setReferenceImage() fails");
    return false;
  }

  double estimatedRotation = 0;
  cv::Vec2f estimatedTranslation;

  if ( !pc.compute(estimatedRotation, estimatedTranslation) ) {
    CF_ERROR("pc.compute() fails");
  }

  switch(_display) {
    case DISPLAY_CURRENT_IMAGE:
      pc.currentFrame().image.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_IMAGE:
      pc.referenceFrame().image.copyTo(image);
      mask.release();
      break;
    case DISPLAY_CURRENT_SPECTRUM:
      fftSpectrumMagnitudeCCS(pc.currentFrame().spec, image, true);
      mask.release();
      break;
    case DISPLAY_REFERENCE_SPECTRUM:
      fftSpectrumMagnitudeCCS(pc.referenceFrame().spec, image, true);
      mask.release();
      break;
    case DISPLAY_CURRENT_ACM:
      pc.currentFrame().acm.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_ACM:
      pc.referenceFrame().acm.copyTo(image);
      mask.release();
      break;
    case DISPLAY_CURRENT_ACM_POLAR:
      pc.currentFrame().acmp.copyTo(image);
      mask.release();
      break;
    case DISPLAY_REFERENCE_ACM_POLAR:
      pc.referenceFrame().acmp.copyTo(image);
      mask.release();
      break;
    case DISPLAY_CURRENT_ACM_POLAR_SPEC:
      fftSpectrumMagnitudeCCS(pc.currentFrame().polarSpec, image, true);
      mask.release();
      break;
    case DISPLAY_REFERENCE_ACM_POLAR_SPEC:
      fftSpectrumMagnitudeCCS(pc.referenceFrame().polarSpec, image, true);
      mask.release();
      break;
    case DISPLAY_POLAR_CROSS_SPECTRUM:
      fftSpectrumMagnitudeCCS(pc.crossSpectrumPolar(), image, true);
      mask.release();
      break;
    case DISPLAY_POLAR_CORRELATION_MAP:
      pc.correlationMapPolar().copyTo(image);
      mask.release();
      break;
    case DISPLAY_rotatedCurrentImage:
      pc.rotatedCurrentImage().copyTo(image);
      mask.release();
      break;
    case DISPLAY_alignedImage:
      // remapImage(_currentImage, image, _referenceImage.size(), estimatedRotation, estimatedTranslation);
      warpImage(_currentImage, image, _referenceImage.size(), estimatedRotation, estimatedTranslation);
      mask.release();
      break;

    case DISPLAY_rotatedCurrentSpec:
      pc.rotatedCurrentSpec().copyTo(image);
      mask.release();
      break;
    case DISPLAY_crossSpectrum:
      fftSpectrumMagnitudeCCS(pc.crossSpectrum(), image, true);
      mask.release();
      break;
    case DISPLAY_correlationMap:
      pc.correlationMap().copyTo(image);
      mask.release();
      break;

  }


  return true;
}

