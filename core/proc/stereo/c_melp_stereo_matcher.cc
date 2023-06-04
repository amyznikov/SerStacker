/*
 * c_melp_stereo_matcher.cc
 *
 *  Created on: Jun 1, 2023
 *      Author: amyznikov
 */

#include "c_melp_stereo_matcher.h"

static constexpr int min_image_size = 16;

c_melp_stereo_matcher::c_melp_stereo_matcher()
{
  // TODO Auto-generated constructor stub

}


bool c_melp_stereo_matcher::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  build_melp_pyramid(left, &lp, min_image_size);
  build_melp_pyramid(right, &rp, min_image_size);

  return true;
}
