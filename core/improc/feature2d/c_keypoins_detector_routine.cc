/*
 * c_keypoins_detector_routine.cc
 *
 *  Created on: Aug 9, 2023
 *      Author: amyznikov
 */

#include "c_keypoins_detector_routine.h"
#include <core/feature2d/feature2d_settings.h>

void c_keypoins_detector_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_SPARSE_FEATURE_DETECTOR_CTRL(ctls, options, "Options", "Options for feature2D detector");
}

bool c_keypoins_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, options_);
    return true;
  }
  return false;
}

void c_keypoins_detector_routine::parameter_changed()
{
  CF_DEBUG("keypoints_detector_.reset()");
  keypoints_detector_.reset();
}


bool c_keypoins_detector_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.needed() && !image.empty() ) {
    if( !keypoints_detector_ && !(keypoints_detector_ = create_sparse_feature_detector(options_)) ) {
      CF_ERROR("create_sparse_feature_detector() fails");
      return false;
    }


    keypoints_.clear();

    try {

      keypoints_detector_->detect(image, keypoints_, mask);

      if( image.channels() == 3 ) {
        image.copyTo(display_);
      }
      else {
        cv::cvtColor(image, display_, cv::COLOR_GRAY2BGR);
      }

      cv::drawKeypoints(display_, keypoints_, display_,
          cv::Scalar::all(-1),
          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      display_.copyTo(image);

    }
    catch( const std::exception &e ) {
      CF_ERROR("keypoints_detector_->detect() fails: %s", e.what());
    }

  }

  return true;
}
