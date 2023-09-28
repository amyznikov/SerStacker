/*
 * c_morph_features_extractor.h
 *
 *  Created on: Sep 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_morph_features_extractor_h__
#define __c_morph_features_extractor_h__

#include <opencv2/opencv.hpp>

class c_morph_features_extractor :
    public cv::Feature2D
{
public:
  typedef c_morph_features_extractor this_class;
  typedef cv::Feature2D base;

  enum MORPH_TYPE {
    MORPH_GRADIENT,
    MORPH_LAPLACIAN
  };

  struct Options
  {
    MORPH_TYPE morph_type = MORPH_GRADIENT;
    double threshold = 10;
    int se_radius = 1;
    // int numOctaves = 4;
  };

  MORPH_TYPE morph_type() const
  {
    return options_.morph_type;
  }

  void set_morph_type(MORPH_TYPE v)
  {
    options_.morph_type = v;
  }

  int se_radius() const
  {
    return options_.se_radius;
  }

  void set_se_radius(int v)
  {
    options_.se_radius = v;
  }

  const Options & options() const
  {
    return options_;
  }

  Options & options()
  {
    return options_;
  }

  void set_options(const Options & v)
  {
    options_ = v;
  }


  c_morph_features_extractor();
  c_morph_features_extractor(const Options & opts);

  static cv::Ptr<this_class> create();
  static cv::Ptr<this_class> create(const Options & opts);

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray _mask) override;


protected:
  Options options_;
};

#endif /* __c_morph_features_extractor_h__ */
