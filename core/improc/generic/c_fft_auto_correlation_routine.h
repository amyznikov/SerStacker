/*
 * c_fft_auto_correlation_routine.h
 *
 *  Created on: Sep 11, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_auto_correlation_routine_h__
#define __c_fft_auto_correlation_routine_h__

#include <core/improc/c_image_processor.h>

class c_fft_auto_correlation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fft_auto_correlation_routine,
      "fft_auto_correlation", "Image auto correlation map with cv::dft()");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_CURRENT_SCALED_IMAGE,
    DISPLAY_CURRENT_SPECTRUM_CART,
    DISPLAY_CURRENT_SPECTRUM_POLAR,
    DISPLAY_CROSS_SPECTRUM_CART,
    DISPLAY_CROSS_SPECTRUM_POLAR,
    DISPLAY_CORRELATION_MAP,
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
  bool ensureInitialized(const cv::Size & expectedFrameSize);
  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  void applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask, const cv::Size & validSize);
  double computeCorrelationMap();

protected: // Controlling parameters
  DISPLAY _display = DISPLAY_CORRELATION_MAP;
  double _downscaleFactor = 4;
  double _gsigma = 0.15;
  int _apodizationSize = 21;

protected: // Cached data
  cv::Size _fftSize;
  std::vector<float> _apodizationLUT;
  cv::Mat1f _distmap;
  cv::Size _currentValidSize;
  cv::Point _currentCropOffset;
  cv::Mat1f _currentImage;
  cv::Mat1f _scaledCurrentImage;
  cv::Mat1b _scaledCurrentMask;
  cv::Mat1f _currentSpectrum;
  cv::Mat1f _crossSpectrum;
  cv::Mat1f _correlationMap;

  bool _initialized = false;
};

#endif /* __c_fft_auto_correlation_roitine_h__ */
