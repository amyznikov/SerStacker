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
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_REFERENCE_IMAGE,
    DISPLAY_CURRENT_SPECTRUM,
    DISPLAY_REFERENCE_SPECTRUM,
    DISPLAY_CURRENT_ACM,
    DISPLAY_REFERENCE_ACM,
    DISPLAY_CURRENT_ACM_POLAR,
    DISPLAY_REFERENCE_ACM_POLAR,
    DISPLAY_CURRENT_ACM_POLAR_COUNTER,
    DISPLAY_REFERENCE_ACM_POLAR_COUNTER,
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
    cv::Mat1f spec; // CCS
    cv::Mat1f acm;
    cv::Mat1f acmPolar;
    cv::Mat1f acmPolarMask;
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
  cv::Mat1f vlap;
  cv::Mat1f bp;

  double _gsimga = 2;
  int _numAngles = 256;
  DISPLAY _display = DISPLAY_CURRENT_ACM;
  bool _maxPooling = false;
  bool _updateReference = true;

};

#endif /* __c_fft_polar_phase_correlate_routine_h__ */
