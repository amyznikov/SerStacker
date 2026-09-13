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
    DISPLAY_FILTER,
    DISPLAY_INVERSE_CROSS,
  };

  struct SpotGeometry
  {
    double peak = 0; // Peak value
    double fwhm_x = 0; // Full width at half maximum along X (in pixels)
    double fwhm_y = 0; // Full width at half maximum along Y (in pixels)
    double angle = 0; // Blur tilt angle (in degrees)
    double eccentricity = 0; // Degree of elongation (0 - circle, 1 - line)
    double radius = 0;
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
  void set_csigma(double v)
  {
    _csigma = v;
    _initialized = false;
  }
  double csigma() const
  {
    return _csigma;
  }
  void set_calpha(double v)
  {
    _calpha = v;
    _initialized = false;
  }
  double calpha() const
  {
    return _calpha;
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
  void generateBandpassFilter();
  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  void applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask, const cv::Size & validSize);
  void computeCorrelationMap();
  bool analyzeSpotGeometry();

protected: // Controlling parameters
  DISPLAY _display = DISPLAY_CORRELATION_MAP;
  double _downscaleFactor = 4;
  double _gsigma = 15;
  double _csigma = 0.5;
  double _calpha = 0.1;
  int _apodizationSize = 21;
  bool _printDebug = false;

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
  cv::Mat1f _bandpassFilter;
  cv::Mat1f _inverseCross;
  cv::Mat1f _autoCrossSpectrum;
  cv::Mat1f _autoCorrelationMap;
  SpotGeometry _spot;
  double _crossSpectrumEnergy = 0;
  double _bandpassFilterNorm = 0;

  bool _initialized = false;
};

#endif /* __c_fft_auto_correlation_roitine_h__ */
