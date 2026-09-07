/*
 * c_alpha_test_routine.h
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_alpha_test_routine_h__
#define __c_alpha_test_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

struct c_phase_correlate_options
{
  bool differentiate = true;
  bool apply_apodization = true;
  bool apply_ramp = true;
  int apodization_radius = 120; // Радиус в пикселях ИСХОДНОГО кадра
};

class c_phase_correlate
{
public:
  c_phase_correlate() = default;

  explicit c_phase_correlate(const c_phase_correlate_options & opts) :
    _opts(opts)
  {
  }

  // Compute outputTranslation such as pos(referenceImage) = outputTranslation + pos(currentImage)
  // Return correlation score or negative value on error
  double compute(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::InputArray referenceImage, cv::InputArray referenceMask,
      cv::Vec2f * outputTranslation);

protected: // internal helpers
  void scaleAndPadToTargetSize(cv::InputArray srcImage, cv::InputArray srcMask,
      cv::Mat & dstImage, cv::Mat & dstMask,
      const cv::Size & targetSize, double scaleFactor);

  void differentiate(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask);
  void generateRampFilter(const cv::Size & size, cv::Mat2f & outputFilter) const;

  void createApodizationWindow(cv::InputArray mask, cv::Mat1f & outputWindow, int kradius);

  cv::Point2f findSubpixelCentroid(const cv::Mat1f & idftResult, cv::Point & outPeakLoc);

public: // data members
  c_phase_correlate_options _opts;

public: // Cache buffers for Zero Allocation at runtime
  cv::Size _cachedSize;
  double _cachedScaleFactor = 0.0;

  cv::Mat _scaledImg1, _scaledMsk1;
  cv::Mat _scaledImg2, _scaledMsk2;
  cv::Mat _gradImg1, _gradImg2;
  cv::Mat _maskedImg1, _maskedImg2;
  cv::Mat _fft1, _fft2, _fftCross;
  cv::Mat2f _rampFilter;
  cv::Mat1f _window1, _window2;
  cv::Mat1f realInverseResult;
};

class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_PREVIOUS_IMAGE,
    DISPLAY_FFT_CROSS,
    DISPLAY_IFFT,
  };


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat prevImage, prevMask, RAMP;
  DISPLAY _display = DISPLAY_CURRENT_IMAGE;
  bool _updatePreviousImage = true;

  c_phase_correlate pc;
};

#endif /* __c_alpha_test_routine_h__ */
