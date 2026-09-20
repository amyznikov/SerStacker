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
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

/**
 * @struct c_phase_correlate_options
 * @brief Configuration parameters for the phase correlation pipeline.
 */
struct c_phase_correlate_options
{
  /** @brief Image downscale factor (must be >= 1.0). Controls the processing resolution. */
  double downscale_factor = 4;

  /** @brief Bandpass filter Gaussian sigma [px]. Defines the target texture characteristic size. */
  double gsigma = 10;

  /** @brief Inverse cross-filter blur parameter for deconvolution/mask-edge compensation. */
  double csigma = 0.5;

  /** @brief Inverse cross-filter scaling/regularization alpha coefficient. */
  double calpha = 0.0;
};

bool serialize_phase_correlate_options(c_config_setting section, bool save,
    c_phase_correlate_options & opts);

inline bool save_settings(c_config_setting section, const c_phase_correlate_options & opts)
{
  return serialize_phase_correlate_options(section, true,
      const_cast<c_phase_correlate_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_phase_correlate_options * opts)
{
  return serialize_phase_correlate_options(section, false, *opts);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_phase_correlate_options> & ctx)
{
  using S = c_phase_correlate_options;
  ctlbind(ctls, "downscale_factor",  ctx(&S::downscale_factor), "");
  ctlbind(ctls, "gsigma [px]:", ctx(&S::gsigma),  "Target texture characteristic size in pixels");
  ctlbind(ctls, "csigma:", ctx(&S::csigma),  "Inverse Cross filter blur");
  ctlbind(ctls, "calpha:", ctx(&S::calpha),  "Inverse Cross filter strength");
}

class c_phase_correlate
{
public:
  // Must be called before pipeline start
  bool setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts);

  // Release internal cache buffers, may be useful for multi-pipeline re-initializators
  void release();

  // Compute phase correlation and return correlation core and translation vector
  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  double compute(cv::Vec2f & outputTranslation);

  // valid after compute()
  double correlationScore() const
  {
    return _correlationScore;
  }

  // valid after compute()
  double peakValue() const
  {
    return _peakValue;
  }

  static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize,
      double downscaleFactor);

public: // public access for debug & visualization purposes
  bool initialized() const {
    return _initialized;
  }
  const cv::Size & fftSize() const {
    return _fftSize;
  }
  const cv::Size & currentValidSize() const {
    return _currentValidSize;
  }
  const cv::Size & referenceValidSize() const {
    return _referenceValidSize;
  }
  const cv::Point & currentCropOffset() const {
    return _currentCropOffset;
  }
  const cv::Point & referenceCropOffset() const {
    return _referenceCropOffset;
  }
  const cv::Mat1f & scaledCurrentImage() const {
    return _scaledCurrentImage;
  }
  const cv::Mat1f & scaledReferenceImage() const {
    return _scaledReferenceImage;
  }
  const cv::Mat1b & scaledCurrentMask() const {
    return _scaledCurrentMask;
  }
  const cv::Mat1b & scaledReferenceMask() const {
    return _scaledReferenceMask;
  }
  const cv::Mat1f & currentSpectrum() const {
    return _currentSpectrum;
  }
  const cv::Mat1f & referenceSpectrum() const {
    return _referenceSpectrum;
  }
  const cv::Mat1f & crossSpectrum() const {
    return _crossSpectrum;
  }
  const cv::Mat1f & correlationMap() const {
    return _correlationMap;
  }
  const cv::Mat1f & bandpassFilter() const {
    return _bandpassFilter;
  }

protected: // internal helpers
  void generateBandpassFilter();
  bool computeCorrelationMap();
  double findSubpixelCentroid(const cv::Mat1f & correlationMap, cv::Point2f & peakPos) const;

protected: // internal data
  cv::Size _fftSize;
  cv::Size _expectedFrameSize;
  double _downscale_factor = 4;
  double _gsigma = 0.1;
  double _csigma = 0.5;
  double _calpha = 0.05;
  double _peakValue = 0;
  double _correlationScore = 0;
  bool _initialized = false;

protected: // Cached data
  cv::Size _currentValidSize, _referenceValidSize;
  cv::Point _currentCropOffset, _referenceCropOffset;
  cv::Mat1f _scaledCurrentImage, _scaledReferenceImage;
  cv::Mat1b _scaledCurrentMask, _scaledReferenceMask;
  cv::Mat1f _currentSpectrum, _referenceSpectrum;
  cv::Mat1f _crossSpectrum, _correlationMap;
  cv::Mat1f _bandpassFilter;
  cv::Mat1f _crossMask;
};


#endif /* __c_phase_correlate_h__ */
