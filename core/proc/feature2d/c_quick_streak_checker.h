/*
 * c_quick_streak_checker.h
 *
 *  Created on: Nov 17, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_quick_streak_checker_h__
#define __c_quick_streak_checker_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_quick_streak_checker_options {
  int pyramidLevel = 2;
  int minTrackLength = 20;
  double noiseSigmaScale = 3;
  double minStraightness = 0.85;
};

bool serialize_quick_streak_checker_options(c_config_setting section, bool save,
    c_quick_streak_checker_options & opts);

inline bool save_settings(c_config_setting section, const c_quick_streak_checker_options & opts)
{
  return serialize_quick_streak_checker_options(section, true,
      const_cast<c_quick_streak_checker_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_quick_streak_checker_options * opts)
{
  return serialize_quick_streak_checker_options(section, false, *opts);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_quick_streak_checker_options> & ctx)
{
  using S = c_quick_streak_checker_options;
  ctlbind(ctls, "PyramidLevel :", ctx(&S::pyramidLevel), "Image downscale pyramid level >= 0");
  ctlbind(ctls, "MinTrackLength:", ctx(&S::minTrackLength), "Minimal streak length in pixels");
  ctlbind(ctls, "NoiseSigmaScale:", ctx(&S::noiseSigmaScale), "Noise threshold scale > 0");
  ctlbind(ctls, "MinStraightness:", ctx(&S::minStraightness), "");
}


/**
* @brief Quick single-pass checker for the presence of at least one linear streak within a binary mask.
* @param _mask  Input binary mask (CV_8UC1; non-zero pixels are considered signal).
* @param minTrackLength Minimum length of a continuous, directional chain (in pixels).
* @return true if at least one track is detected
*/
class c_quick_streak_checker
{
public:

  bool check_for_streaks(const c_quick_streak_checker_options & opts, cv::InputArray src,
      cv::InputArray mask = cv::noArray(),
      cv::Vec4i* outTrack = nullptr);

  const cv::Mat & tophat() const
  {
    return _tophat;
  }

  const cv::Mat & topmask() const
  {
    return _topmask;
  }

  double median() const
  {
    return _median;
  }

  double sigma() const
  {
    return _sigma;
  }

  double threshold() const
  {
    return _threshold;
  }

protected:
  cv::Mat _tophat;
  cv::Mat _topmask;
  std::vector<uint64_t> _scratchBuf;
  double _median = 0;
  double _sigma = 0;
  double _threshold = 0;
};

#endif /* __c_quick_streak_checker_h__ */
