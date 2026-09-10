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
#include <core/proc/image_registration/c_phase_correlate.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>



class c_alpha_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_alpha_test_routine,
      "alpha_test", "Alpha Test");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_REFERENCE_IMAGE,
    DISPLAY_BLEND_IMAGE,
    DISPLAY_SHIFTED_CURRENT_IMAGE,

    DISPLAY_CURRENT_SCALED_IMAGE,
    DISPLAY_REFERENCE_SCALED_IMAGE,
    DISPLAY_BLEND_SCALED_IMAGE,

    DISPLAY_CROSS_MODULE,
    DISPLAY_CROSS_PHASE,

    DISPLAY_CROSS_pureCross,
    DISPLAY_CROSS_weightMap,
    DISPLAY_CROSS_gradX,
    DISPLAY_CROSS_gradY,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  void set_downscaleFactor(double v)
  {
    _downscaleFactor = v;
    _initialized = false;
  }
  double downscaleFactor() const
  {
    return _downscaleFactor;
  }
  void set_gsigma(double v)
  {
    _gsigma = v;
    _initialized = false;
  }
  double gsigma() const
  {
    return _gsigma;
  }
  void set_apodizationSize(int v)
  {
    _apodizationSize = v;
    _initialized = false;
  }
  int apodizationSize() const
  {
    return _apodizationSize;
  }

protected:
  bool reinitialize(const cv::Size & expectedFrameSize);
  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  bool setCurrentImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  void applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask, const cv::Size & validSize);
  bool compute(cv::Vec2f & outputTranslation);

protected: // Controlling parameters
  DISPLAY _display = DISPLAY_CURRENT_IMAGE;

  double _downscaleFactor = 4;
  double _gsigma = 0.15;
  int _apodizationSize = 21;
  bool _updateReferenceImage = true;

protected: // Cached data
  cv::Size _fftSize;
  std::vector<float> _apodizationLUT;
  cv::Size _currentValidSize, _referenceValidSize;
  cv::Point _currentCropOffset, _referenceCropOffset;
  cv::Mat1f _currentImage, _referenceImage;
  cv::Mat1f _scaledCurrentImage, _scaledReferenceImage;
  cv::Mat1b _scaledCurrentMask, _scaledReferenceMask;
  cv::Mat2f _currentSpectrum, _referenceSpectrum;
  cv::Mat2f _crossSpectrum;
  cv::Mat1f _distmap;
  cv::Mat2f _Px_phase_acc, _Py_phase_acc;
  cv::Mat2f _pureCross;
  cv::Mat1f _weightMap;
  cv::Mat2f _gradX, _gradY;

  bool _initialized = false;
};

// c_phase_correlate pc;

#endif /* __c_alpha_test_routine_h__ */
