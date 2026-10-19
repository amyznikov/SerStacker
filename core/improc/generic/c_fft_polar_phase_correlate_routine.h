/*
 * c_fft_polar_phase_correlate_routine.h
 *
 *  Created on: Oct 16, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_polar_phase_correlate_routine_h__
#define __c_fft_polar_phase_correlate_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_polar_phase_correlate.h>

class c_fft_polar_phase_correlate_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_polar_phase_correlate_routine,
      "fft_polar_phase_correlate", "Test");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_REFERENCE_IMAGE,
    DISPLAY_CURRENT_SPECTRUM,
    DISPLAY_REFERENCE_SPECTRUM,
    DISPLAY_CURRENT_ACM,
    DISPLAY_REFERENCE_ACM,
    DISPLAY_CURRENT_ACM_POLAR,
    DISPLAY_REFERENCE_ACM_POLAR,
    DISPLAY_CURRENT_ACM_POLAR_SPEC,
    DISPLAY_REFERENCE_ACM_POLAR_SPEC,
    DISPLAY_POLAR_CROSS_SPECTRUM,
    DISPLAY_POLAR_CORRELATION_MAP,

    DISPLAY_rotatedCurrentImage,
    DISPLAY_rotatedCurrentSpec,
    DISPLAY_crossSpectrum,
    DISPLAY_correlationMap,
  };

  void set_gsigma(double v)
  {
    opts.gsigma1 = v;
    _initialized = false;
  }
  double gsigma() const
  {
    return opts.gsigma1;
  }

  void set_gsigma2(double v)
  {
    opts.gsigma2 = v;
    _initialized = false;
  }
  double gsigma2() const
  {
    return opts.gsigma2;
  }

  void set_apodization(double v)
  {
    opts.apodization = v;
    _initialized = false;
  }

  double apodization() const
  {
    return opts.apodization;
  }

  void set_whitenPolarSpecs(bool v)
  {
    opts.whitenPolarSpecs = v;
    _initialized = false;
  }

  bool whitenPolarSpecs() const
  {
    return opts.whitenPolarSpecs;
  }

  void set_whitenTransSpecs(bool v)
  {
    opts.whitenTransSpecs = v;
    _initialized = false;
  }

  bool whitenTransSpecs() const
  {
    return opts.whitenTransSpecs;
  }

  void set_maxPooling(bool v)
  {
    opts.maxPooling = v;
    _initialized = false;
  }

  bool maxPooling() const
  {
    return opts.maxPooling;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

  static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize,
      double downscaleFactor);

protected:
  bool reinitialize(const cv::Size & expectedFrameSize);
  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);

protected:
  c_polar_phase_correlate_options opts;
  c_polar_phase_correlate pc;

  cv::Mat _currentImage, _referenceImage;
  cv::Mat _currentMask, _referenceMask;
  bool _initialized = false;

  DISPLAY _display = DISPLAY_CURRENT_ACM;
  bool _printScores = false;
  bool _printDiagnostics = false;
  bool _updateReference = true;
};

#endif /* __c_fft_polar_phase_correlate_routine_h__ */
