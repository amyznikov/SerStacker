/*
 * c_phase_correlate.h
 *
 *  Created on: Sep 7, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_phase_correlate_h__
#define __c_phase_correlate_h__

#include <opencv2/opencv.hpp>

struct c_phase_correlate_options
{
  double downscale_factor = 4;
  double gsigma = 0.1;
  int apodization_size = 21;
};

class c_phase_correlate
{
public:
  // Must be called before pipeline start
  bool setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts);

  // Release internal cache buffers, may be useful for multi-pipeline re-initializators
  void release();

  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  bool setCurrentImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  double compute(cv::Vec2f & outputTranslation);

protected: // internal helpers
  void applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask, const cv::Size & validSize);
  double computeCorrelationMap();
  cv::Point2f findSubpixelCentroid(const cv::Mat1f & idftResult, cv::Point & outPeakLoc);

protected: // internal data
  double _downscale_factor = 4;
  double _gsigma = 0.1;
  int _apodization_size = 21;
  cv::Size _fftSize;

public: // Made temporary public for debug & visualization purposes only
  std::vector<float> _apodizationLUT;
  cv::Size _currentValidSize, _referenceValidSize;
  cv::Point _currentCropOffset, _referenceCropOffset;
  cv::Mat1f _scaledCurrentImage, _scaledReferenceImage;
  cv::Mat1b _scaledCurrentMask, _scaledReferenceMask;
  cv::Mat1f _currentWindow, _referenceWindow;
  cv::Mat1f _currentSpectrum, _referenceSpectrum;
  cv::Mat1f _crossSpectrum, _correlationMap;
  cv::Mat1f _distmap;
};

#endif /* __c_phase_correlate_h__ */
