/*
 * c_regular_stereo_matcher.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "c_regular_stereo_matcher.h"
#include <core/debug.h>

c_regular_stereo_matcher::c_regular_stereo_matcher()
{
}


void c_regular_stereo_matcher::set_max_disparity(int v)
{
  max_disparity_ = v;
}

int c_regular_stereo_matcher::max_disparity() const
{
  return max_disparity_;
}

void c_regular_stereo_matcher::set_max_scale(int v)
{
  max_scale_ = v;
}

int c_regular_stereo_matcher::max_scale() const
{
  return max_scale_;
}

bool c_regular_stereo_matcher::match(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::InputArray referenceImage, cv::InputArray referenceMask,
    cv::Mat2f & outputMatches)
{


  return false;

}
