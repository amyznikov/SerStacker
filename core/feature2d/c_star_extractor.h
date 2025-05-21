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

class c_star_extractor:
    public cv::Feature2D
{
public:
  typedef c_star_extractor this_class;
  typedef cv::Feature2D bae;

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

  c_star_extractor();

  static cv::Ptr<this_class> create();

  void set_median_filter_size(int v)
  {
    _median_filter_size = v;
  }

  int median_filter_size() const
  {
    return _median_filter_size;
  }

  void set_sigma1(double v)
  {
    _sigma1 = v;
  }

  double sigma1() const
  {
    return _sigma1;
  }

  void set_sigma2(double v)
  {
    _sigma2 = v;
  }

  double sigma2() const
  {
    return _sigma2;
  }

  void set_noise_blur(double v)
  {
    _noise_blur = v;
  }

  double noise_blur() const
  {
    return _noise_blur;
  }

  void set_noise_threshold(double v)
  {
    _noise_threshold = v;
  }

  double noise_threshold() const
  {
    return _noise_threshold;
  }

  void set_min_pts(int v)
  {
    _min_pts = v;
  }

  int min_pts() const
  {
    return _min_pts;
  }

  void set_min_b(double v)
  {
    _min_b = v;
  }

  double min_b() const
  {
    return _min_b;
  }

  void set_min_ba_ratio(double v)
  {
    _min_ba_ratio = v;
  }

  double min_ba_ratio() const
  {
    return _min_ba_ratio;
  }

  void set_min_score(double v)
  {
    _min_score = v;
  }

  double min_score() const
  {
    return _min_score;
  }

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray _mask) final;

protected:
  // detector opts
  int _median_filter_size = 0;
  double _sigma1 = 2;
  double _sigma2 = 5;
  double _noise_blur = 100;
  double _noise_threshold = 7;

  // filters
  double _min_score = 0.4;
  double _min_b = 1;
  double _min_ba_ratio = 0.7;
  int _min_pts = 5;

  // work arrays
  cv::Mat _image;
  cv::Mat1f _noisemap;
  cv::Mat1i _labels;
  cv::Mat1b _cc;
  std::vector<Blob> _blobs;


};

#endif /* __c_star_extractor_h__ */
