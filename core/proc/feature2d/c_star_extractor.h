/*
 * c_star_extractor.h
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_star_extractor_h__
#define __c_star_extractor_h__

#include <opencv2/opencv.hpp>
#include <core/proc/feature2d/c_simple_star_detector.h>

class c_star_extractor:
    public cv::Feature2D
{
public:
  typedef c_star_extractor this_class;
  typedef cv::Feature2D base;
  using Blob = c_simple_star_detector::Blob;
  using Options = c_simple_star_detector_options;

  c_star_extractor();

  static cv::Ptr<this_class> create();
  static cv::Ptr<this_class> create(const c_simple_star_detector_options & opts);

  void set_opts(const c_simple_star_detector_options & opts);
  const c_simple_star_detector_options & opts() const;

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray _mask) final;

  const c_simple_star_detector & detector() const
  {
    return _detector;
  }

  const c_simple_star_detector_options & options() const
  {
    return _detector.options();
  }

  void set_options(const c_simple_star_detector_options & opts)
  {
    _detector.set_options(opts);
  }

protected:
  c_simple_star_detector _detector;
};

#endif /* __c_star_extractor_h__ */
