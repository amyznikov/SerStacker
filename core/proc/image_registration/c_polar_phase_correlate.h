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
  double downscale_factor = 1;
  double gsigma1 = 2;
  double gsigma2 = 2;
  double gsigma3 = 2;
  double apodization = 0;
  bool maxPooling = false;
  bool whitenPolarSpecs = true;
  bool whitenTransSpecs = true;
  bool estimateRotation = false;
  bool estimateTranslation = true;
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

  ctlbind(ctls, "downscale_factor",  ctx(&S::downscale_factor),
      "Image downscale factor >= 1");

  ctlbind(ctls, "gsigma1 [px]:", ctx(&S::gsigma1),
      "Rotation Estimate: Bandpass filter used to compute image AUTO correlation map before polar conversion");

  ctlbind(ctls, "gsigma2 [px]:", ctx(&S::gsigma2),
      "Rotation Estimate: Bandpass filter used to compute CROSS correlation of Polar maps");

  ctlbind(ctls, "gsigma3 [px]:", ctx(&S::gsigma3),
      "Translation Estimate: Bandpass filter used to compute CROSS correlation of reference and current derotated image");

  ctlbind(ctls, "apodization:", ctx(&S::apodization),
      "Optional Apodization window parameter [0..1]");

  ctlbind(ctls, "maxPooling:", ctx(&S:: maxPooling),
      "Set true for max pooling - useful for low-contrast deep fields to prevent loss of faint stars");

  ctlbind(ctls, "whitenPolarSpecs:", ctx(&S:: whitenPolarSpecs),
      "Set true to apply pure phase correlation for rotation estimation");

  ctlbind(ctls, "whitenTransSpecs:", ctx(&S:: whitenTransSpecs),
      "Set true to apply pure phase correlation for translation estimation");

  ctlbind(ctls, "EstimateRotation:", ctx(&S::estimateRotation),
      "Enable rotation estimation");

  ctlbind(ctls, "EstimateTranslation:", ctx(&S::estimateTranslation),
      "Enable translation estimation");
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
    cv::Rect srcROI;
    cv::Rect dstROI;
    //cv::Point cropOffset;
    cv::Mat1f polarSpec;
  };

  static constexpr int numAngles = 512;

  // Must be called before pipeline start
  bool setup(const cv::Size & expectedFrameSize, c_polar_phase_correlate_options & opts);

  // Release internal cache buffers, may be useful for multi-pipeline re-initializators
  void release();

  bool setCurrentImage(cv::InputArray currentImage, cv::InputArray currentMask);
  bool setReferenceImage(cv::InputArray referenceImage, cv::InputArray referenceMask);
  bool compute(double & outputEstimatedRotation, cv::Vec2f & outputEstimatedTranslation);


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
    return _derotatedCurrentImage;
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

  static cv::Size computeFFTPackSize(const cv::Size & expectedFrameSize, double downscaleFactor);

protected: // internal helpers
  void generateBandpassFilter(cv::Mat1f & filter, const cv::Size & size, double sigma, bool squared) const;
  bool setupInputFrame(cv::InputArray srcImage, cv::InputArray srcMask, Frame * frame);
  bool computeOrientation(double & outAngleRadian);
  bool computeTranslation(double orientationAngleRadian, cv::Vec2f & outTranslation);

  bool estimateRotationPeak(const cv::Mat1f & correlationMap, double crossEnergyScale,
      cv::Point2f & outPeakPos, double & outPeakVal) const;

  bool estimateTranslationPeak(const cv::Mat1f & correlationMap, double crossEnergyScale,
      cv::Point2f & outPeakPos, double & outPeakVal) const;

  bool estimatePeakLocation(const cv::Mat1f & correlationMap,double crossEnergyScale,
      double threshold, double gsigma,
      cv::Point2f & outPeakPos, double & outPeakVal,
      const cv::Rect & searchROI = cv::Rect()) const;

protected: // internal data
  cv::Size _fftSize;
  cv::Size _expectedFrameSize;
  double _downscale_factor = 1;
  double _gsigma1 = 2;
  double _gsigma2 = 2;
  double _gsigma3 = 2;
  double _aalpha = 0.2;
  double _correlationScore = 0;
  bool _maxPooling = false;
  bool _whitenPolarSpecs = true;
  bool _whitenTransSpecs = true;
  bool _estimateRotation = false;
  bool _estimateTranslation = true;
  bool _initialized = false;

protected: // Cached data
  Frame _referenceFrame;
  Frame _currentFrame;
  cv::Mat1f _apodizationWindow;
  cv::Mat1f _vlap2;
  cv::Mat1f _bp1;
  cv::Mat1f _bp2;
  cv::Mat1f _bp3;
  cv::Mat1f _crossSpectrumPolar;
  cv::Mat1f _correlationMapPolar;
  cv::Mat1f _derotatedCurrentImage;
  cv::Mat1f _rotatedCurrentSpec;
  cv::Mat1f _crossSpectrum;
  cv::Mat1f _correlationMap;
};

#endif /* __c_fft_polar_phase_correlate_h__ */
