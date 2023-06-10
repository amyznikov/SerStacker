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

  c_melp_stereo_matcher();

  bool compute(cv::InputArray left, cv::InputArray right,
      cv::OutputArray disparity);

protected:
};

#endif /* __c_melp_stereo_matcher_h__ */
