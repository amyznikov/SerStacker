/*
 * c_dct_auto_correlation_routine.h
 *
 *  Created on: Oct 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dct_auto_correlation_routine_h__
#define __c_dct_auto_correlation_routine_h__

#include <core/improc/c_image_processor.h>

class c_dct_auto_correlation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dct_auto_correlation_routine,
      "dct_auto_correlation", "Image auto correlation map with cv::dct()");

  enum DISPLAY {
    DISPLAY_CURRENT_IMAGE,
    DISPLAY_CURRENT_SCALED_IMAGE,
    DISPLAY_CURRENT_SPECTRUM_DCT,
    DISPLAY_CORRELATION_MAP,
    DISPLAY_FILTER,
  };

  struct SpotGeometry
  {
    double peak = 0; // Peak value
    double fwhm_x = 0; // FWHM along X (in pixels)
    double fwhm_y = 0; // FWHM along Y (in pixels)
    double angle = 0; // Tilt (in degrees)
    double eccentricity = 0; // Eccentricity
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

protected:
  bool ensureInitialized(const cv::Size & expectedFrameSize);
  void generateBandpassFilter();
  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  void computeCorrelationMap();
  bool analyzeSpotGeometry();

protected:
  DISPLAY _display = DISPLAY_CORRELATION_MAP;
  double _downscaleFactor = 4;
  double _gsigma = 5;
  bool _printDebug = false;

protected:
  cv::Size _dctSize;
  cv::Size _currentValidSize;
  cv::Point _currentCropOffset;
  cv::Mat1f _currentImage;
  cv::Mat1f _scaledCurrentImage;
  cv::Mat1b _scaledCurrentMask;
  cv::Mat1f _currentSpectrum;
  cv::Mat1f _bandpassFilter;
  cv::Mat1f _autoCrossSpectrum;
  cv::Mat1f _autoCorrelationMap;
  SpotGeometry _spot;
  double _crossSpectrumEnergy = 0;

  bool _initialized = false;
};

#endif /* __c_dct_auto_correlation_routine_h__ */
