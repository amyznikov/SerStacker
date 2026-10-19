/*
 * c_fft_polar_phase_correlate.h
 *
 *  Created on: Oct 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fft_polar_phase_correlate_h__
#define __c_fft_polar_phase_correlate_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_polar_phase_correlate_options
{
  double gsigma1 = 2;
  double gsigma2 = 3;
  double apodization = 0;
  bool maxPooling = false;
  bool whitenPolarSpecs = true;
  bool whitenTransSpecs = true;
};

bool serialize_polar_phase_correlate_options(c_config_setting section, bool save,
    c_polar_phase_correlate_options & opts);

inline bool save_settings(c_config_setting section, const c_polar_phase_correlate_options & opts)
{
  return serialize_polar_phase_correlate_options(section, true,
      const_cast<c_polar_phase_correlate_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_polar_phase_correlate_options * opts)
{
  return serialize_polar_phase_correlate_options(section, false, *opts);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_polar_phase_correlate_options> & ctx)
{
  using S = c_polar_phase_correlate_options;
  ctlbind(ctls, "gsigma1  [px]:", ctx(&S::gsigma1),  "Bandpass filter radius in space domain [px]. Defines the target texture characteristic size");
  ctlbind(ctls, "gsigma2 [px]:", ctx(&S::gsigma2),  "Bandpass filter radius in polar ACM domain [px]");
  ctlbind(ctls, "apodization:", ctx(&S::apodization),  "Optional Apodization window parameter [0..1]");
  ctlbind(ctls, "maxPooling:", ctx(&S:: maxPooling),  "Set true for max pooling");
  ctlbind(ctls, "whitenPolarSpecs:", ctx(&S:: whitenPolarSpecs),  "Set true to apply pure phase correlation for rotation estimation");
  ctlbind(ctls, "whitenTransSpecs:", ctx(&S:: whitenTransSpecs),  "Set true to apply pure phase correlation for translation estimation");
}

class c_polar_phase_correlate
{
public:
  struct Frame {
    cv::Mat1f image;
    cv::Mat1b mask;
    cv::Mat1f spec;
    cv::Mat1f autospec;
    cv::Mat1f acm;
    cv::Mat1f acmp;
    cv::Mat1f acmpm;
    cv::Rect validROI;
    cv::Point cropOffset;
    cv::Mat1f polarSpec;
  };

  // Must be called before pipeline start
  bool setup(const cv::Size & expectedFrameSize, c_polar_phase_correlate_options & opts);

  // Release internal cache buffers, may be useful for multi-pipeline re-initializators
  void release();

  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  bool compute();

  const Frame & referenceFrame() const
  {
    return _referenceFrame;
  }
  const Frame & currentFrame() const
  {
    return _currentFrame;
  }
  const cv::Mat1f & crossSpectrumPolar() const
  {
    return _crossSpectrumPolar;
  }
  const cv::Mat1f & correlationMapPolar() const
  {
    return _correlationMapPolar;
  }
  const cv::Mat1f & rotatedCurrentImage() const
  {
    return _rotatedCurrentImage;
  }
  const cv::Mat1f & rotatedCurrentSpec() const
  {
    return _rotatedCurrentSpec;
  }
  const cv::Mat1f & crossSpectrum() const
  {
    return _crossSpectrum;
  }
  const cv::Mat1f & correlationMap() const
  {
    return _correlationMap;
  }

  static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize);

protected: // internal helpers
  void generateBandpassFilter1();
  void generateBandpassFilter2(const cv::Size & polarSize);
  bool setupInputFrame(cv::InputArray srcImage, cv::InputArray srcMask, Frame * frame);
  double computeOrientation();
  bool computeTranslation(double orientation_angle, cv::Point2f & outTranslation);
  bool estimatePolarPeak(const cv::Mat1f & correlationMap, cv::Point2f & outPeakPos, double & outPeakVal) const;

protected: // internal data
  cv::Size _fftSize;
  cv::Size _expectedFrameSize;
  double _gsigma1 = 2;
  double _gsigma2 = 2;
  double _aalpha = 0.2;
  double _correlationScore = 0;
  bool _maxPooling = false;
  bool _whitenPolarSpecs = true;
  bool _whitenTransSpecs = true;
  bool _initialized = false;

protected: // Cached data
  Frame _referenceFrame;
  Frame _currentFrame;
  cv::Mat1f _apodizationWindow;
  cv::Mat1f _vlap1;
  cv::Mat1f _vlap2;
  cv::Mat1f _bp1;
  cv::Mat1f _bp2;
  cv::Mat1f _crossSpectrumPolar;
  cv::Mat1f _correlationMapPolar;
  cv::Mat1f _rotatedCurrentImage;
  cv::Mat1f _rotatedCurrentSpec;
  cv::Mat1f _crossSpectrum;
  cv::Mat1f _correlationMap;
};

#endif /* __c_fft_polar_phase_correlate_h__ */
