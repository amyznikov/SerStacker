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

class c_fft_polar_phase_correlate_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_polar_phase_correlate_routine,
      "fft_polar_phase_correlate", "Test");

  enum DISPLAY {
    DISPLAY_CURRENT_SPECTRUM,
    DISPLAY_REFERENCE_SPECTRUM,
    DISPLAY_CUREENT_SPECTRUM_LP,
    DISPLAY_REFERENCE_SPECTRUM_LP,
    DISPLAY_CORRELATION_MAP,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

  static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize,
      double downscaleFactor);

protected:
  struct Frame {
    cv::Mat1f image;
    cv::Mat1b mask;
    cv::Mat1f imageSpec; // Magnitude in cartesian
    cv::Mat1f imageSpecLogPolar; // Magnitude in log polar
    cv::Mat1f imageSpecLogPolarSpec; // CCS
    cv::Size validSize;
    cv::Point cropOffset;
  };

  bool setupInputImage(cv::InputArray srcImage, cv::InputArray srcMask,const cv::Mat1f & VLAP,
      cv::Mat1f & outScaledImage, cv::Mat1b & outScaledMask,
      cv::Size & outValidSize, cv::Point & outCropOffset,
      cv::Mat1f & outSingleSpectrum) const;

  bool setupInputFrame(cv::InputArray srcImage, cv::InputArray srcMask,
      Frame * frame) const;

  static void generateBandpassFilter1(cv::Mat1f & outputFilter,
      const cv::Size & fftSize,
      double gsigma);

  static void generateBandpassFilter2(cv::Mat1f & outputFilter,
      const cv::Size & fftSize,
      double gsigma);

protected:
  Frame referenceFrame;
  Frame currentFrame;
  cv::Mat1f vlap1, vlap2;
  cv::Mat1f bp1, bp2;

  double _downscale_factor = 1;
  double _gsimga1 = 3;
  double _gsimga2 = 3;
  const int numRadii = 256;
  int _numAngles = 256;
  DISPLAY _display = DISPLAY_CORRELATION_MAP;
  bool _updateReference = true;

};

#endif /* __c_fft_polar_phase_correlate_routine_h__ */
