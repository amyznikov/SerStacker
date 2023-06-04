/*
 * c_melp_stereo_matcher.h
 *
 *  Created on: Jun 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_melp_stereo_matcher_h__
#define __c_melp_stereo_matcher_h__

#include <core/proc/laplacian_pyramid.h>

class c_melp_stereo_matcher
{
public:

  // 3x3 BGR pixels, 32 bytes = 256 bits
  struct descriptor
  {
    uint8_t a[32];
  };


  c_melp_stereo_matcher();

  bool compute(cv::InputArray left, cv::InputArray right,
      cv::OutputArray disparity);

  //static bool build_melp_pyramid()

protected:
  c_melp_pyramid lp, rp;
};

#endif /* __c_melp_stereo_matcher_h__ */
