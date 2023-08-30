/*
 * c_epipolar_matcher.cc
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#include "c_epipolar_matcher.h"
#include <core/debug.h>

c_epipolar_matcher::c_epipolar_matcher()
{
  // TODO Auto-generated constructor stub

}


bool c_epipolar_matcher::serialize(c_config_setting settings, bool save)
{

  return true;
}


bool c_epipolar_matcher::compute_block_array(cv::InputArray image, cv::InputArray mask,
    c_block_array * output_block_array) const
{

  if( image.type() != CV_8UC3 ) {
    CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): CV_8UC3 image requited");
    return false;
  }

  if( !mask.empty() ) {
    if( mask.type() != CV_8UC1 ) {
      CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): mask type must be CV_8UC1");
      return false;
    }

    if( mask.size() != image.size() ) {
      CF_ERROR("ERROR: Invalid input in c_epipolar_matcher::compute_block_array(): mask and image sizes must match");
      return false;

    }
  }

  const cv::Mat3b src =
      image.getMat();

  const cv::Mat1b src_mask =
      mask.getMat();




  return true;
}
