/*
 * c_melp_stereo_matcher_routine.cc
 *
 *  Created on: Jun 10, 2023
 *      Author: amyznikov
 */

#include "c_melp_stereo_matcher_routine.h"

void c_melp_stereo_matcher_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
}

bool c_melp_stereo_matcher_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    return true;
  }
  return false;
}

bool c_melp_stereo_matcher_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat images[2];

  cv::Mat src_image =
      image.getMat();

  cv::Mat src_mask =
      mask.getMat();

  const cv::Size src_size =
      src_image.size();

  const cv::Size s(src_size.width / 2, src_size.height);

  const cv::Rect ROI[2] = {
      cv::Rect(0, 0, s.width, s.height),
      cv::Rect(s.width, 0, s.width, s.height),
  };

  for( int i = 0; i < 2; ++i ) {
    images[i] = src_image(ROI[i]);
  }

  cv::Mat D;

  if( !m.compute(images[0], images[1], D) ) {
    CF_ERROR("m.compute() fails");
    return false;
  }

  D.copyTo(image);
  if ( mask.needed() ) {
    mask.release();
  }

  return true;
}
