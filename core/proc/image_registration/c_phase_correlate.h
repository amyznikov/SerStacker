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

struct c_phase_correlate_options
{
  double downscale_factor = 4;
  double gsigma = 0.1;
  int apodization_size = 21;
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
  ctlbind(ctls, "gsigma", ctx(&S::gsigma),  "");
  ctlbind(ctls, "apodization_size",  ctx(&S::apodization_size), "");
}

class c_phase_correlate
{
public:
  // Must be called before pipeline start
  bool setup(const cv::Size & expectedFrameSize, c_phase_correlate_options & opts);

  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  bool setCurrentImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  double compute(cv::Vec2f & outputTranslation);

  // Release internal cache buffers, may be useful for multi-pipeline re-initializators
  void release();

  void setGSigma(double gsigma)
  {
    _gsigma = gsigma;
  }

  double gsigma() const
  {
    return _gsigma;
  }

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
  const cv::Size referenceValidSize() const {
    return _referenceValidSize;
  }
  const cv::Point currentCropOffset() const {
    return _currentCropOffset;
  }
  const cv::Point referenceCropOffset() const {
    return _referenceCropOffset;
  }
  const cv::Mat1f scaledCurrentImage() const {
    return _scaledCurrentImage;
  }
  const cv::Mat1f scaledReferenceImage() const {
    return _scaledReferenceImage;
  }
  const cv::Mat1b scaledCurrentMask() const {
    return _scaledCurrentMask;
  }
  const cv::Mat1b scaledReferenceMask() const {
    return _scaledReferenceMask;
  }
  const cv::Mat1f currentSpectrum() const {
    return _currentSpectrum;
  }
  const cv::Mat1f referenceSpectrum() const {
    return _referenceSpectrum;
  }
  const cv::Mat1f crossSpectrum() const {
    return _crossSpectrum;
  }
  const cv::Mat1f correlationMap() const {
    return _correlationMap;
  }
  const cv::Mat1f distmap() const {
    return _distmap;
  }

protected: // internal helpers
  void applyApodization(cv::Mat1f & scaledImage, const cv::Mat1b & scaledMask, const cv::Size & validSize);
  double computeCorrelationMap();
  cv::Point2f findSubpixelCentroid(const cv::Mat1f & idftResult, cv::Point & outPeakLoc);

protected: // internal data
  double _downscale_factor = 4;
  double _gsigma = 0.1;
  int _apodization_size = 21;
  cv::Size _fftSize;
  bool _initialized = false;

protected: // Cache data
  std::vector<float> _apodizationLUT;
  cv::Size _currentValidSize, _referenceValidSize;
  cv::Point _currentCropOffset, _referenceCropOffset;
  cv::Mat1f _scaledCurrentImage, _scaledReferenceImage;
  cv::Mat1b _scaledCurrentMask, _scaledReferenceMask;
  cv::Mat1f _currentSpectrum, _referenceSpectrum;
  cv::Mat1f _crossSpectrum, _correlationMap;
  cv::Mat1f _distmap;
};

#endif /* __c_phase_correlate_h__ */
