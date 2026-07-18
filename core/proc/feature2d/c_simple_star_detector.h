/*
 * c_simple_star_detector.h
 *
 *  Created on: Jul 15, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_simple_star_detector_h__
#define __c_simple_star_detector_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_simple_star_detector_options
{
  double sigma = 2;
  double weight_decay = 1;
  double kmad = 9;
  int lvls = 5;
  int se_radius = 2;
  double min_a = 0.7;
  double max_a = 10;
  double max_elongation = 1.5;
  double min_compactness = 3;
};

bool serialize_simple_star_detector_options(c_config_setting section, bool save,
    c_simple_star_detector_options & opts);

inline bool save_settings(c_config_setting section, const c_simple_star_detector_options & opts)
{
  return serialize_simple_star_detector_options(section, true,
      const_cast<c_simple_star_detector_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_simple_star_detector_options * opts)
{
  return serialize_simple_star_detector_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_simple_star_detector_options> & ctx)
{
  using S = c_simple_star_detector_options;

  ctlbind(ctls, "sigma", ctx(&S::sigma), "");
  ctlbind(ctls, "weight_decay", ctx(&S::weight_decay), "");
  ctlbind(ctls, "kmad", ctx(&S::kmad), "");
  ctlbind(ctls, "se_radius", ctx(&S::se_radius), "");
  ctlbind(ctls, "lvls", ctx(&S::lvls), "");
  ctlbind(ctls, "min_a", ctx(&S::min_a), "");
  ctlbind(ctls, "max_a", ctx(&S::max_a), "");
  ctlbind(ctls, "max_elongation", ctx(&S::max_elongation), "");
  ctlbind(ctls, "min_compactness", ctx(&S::min_compactness), "");

}

class c_simple_star_detector
{
public:
  typedef c_simple_star_detector this_class;

  struct Blob {
    double x = 0;
    double y = 0;
    double x2 = 0;
    double y2 = 0;
    double xy = 0;
    double I = 0;
    double n = 0;
    double a;
    double b;
    double theta;
  };

  c_simple_star_detector()
  {
  }

  explicit c_simple_star_detector(const c_simple_star_detector_options & opts) :
      _opts(opts)
  {
  }

  const c_simple_star_detector_options & options() const
  {
    return _opts;
  }

  c_simple_star_detector_options & options()
  {
    return _opts;
  }

  void set_options(const c_simple_star_detector_options & opts)
  {
    _opts = opts;
  }

  const std::vector<Blob> & detected_blobs() const
  {
    return _final_blobs;
  }

  const cv::Mat1f &get_dog() const
  {
    return dog;
  }

  const cv::Mat1b &get_cc() const
  {
    return cc;
  }

  const std::vector<Blob> & detect(cv::InputArray image,
      cv::InputArray mask);

protected:
  c_simple_star_detector_options _opts;
  cv::Mat1f dog;
  cv::Mat1b cc;
  std::vector<Blob> _final_blobs;
  cv::Mat1f G;
  cv::Mat1b SE;
  double gsigma = 0;
};

#endif /* __c_simple_star_detector_h__ */
