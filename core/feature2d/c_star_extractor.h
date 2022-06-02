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

  c_star_extractor(int numOctaves = 4);

  static cv::Ptr<this_class> create(int numOctaves = 4);

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray _mask) override;
};

#endif /* __c_star_extractor_h__ */
