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
  BIND_PCTRL(ctls, black_background, "");
}

bool c_keypoins_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, options_);
    SERIALIZE_PROPERTY(settings, save, *this, black_background);
    return true;
  }
  return false;
}

void c_keypoins_detector_routine::parameter_changed()
{
  keypoints_detector_.reset();
}

bool c_keypoins_detector_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  try {

    if( image.needed() && !image.empty() ) {

      if( !keypoints_detector_ && !(keypoints_detector_ = create_sparse_feature_detector(options_)) ) {
        CF_ERROR("create_sparse_feature_detector() fails");
        return false;
      }

      keypoints_.clear();

      keypoints_detector_->detect(image, keypoints_, mask);

      if( options_.max_keypoints > 0 && keypoints_.size() > options_.max_keypoints ) {

        std::sort(keypoints_.begin(), keypoints_.end(),
            [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
              return prev.response > next.response;
            });

        keypoints_.erase(keypoints_.begin() + options_.max_keypoints,
            keypoints_.end());
      }

      if( black_background_ ) {
        display_.create(image.size(), CV_MAKETYPE(image.depth(), 3));
        display_.setTo(cv::Scalar::all(0));
      }
      else if( image.channels() == 3 ) {
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

    return true;
  }

  catch( const std::exception & e ) {
    CF_ERROR("keypoints_detector_->detect() fails: %s", e.what());
  }
  catch( ... ) {
    CF_ERROR("UNknown exception in c_keypoins_detector_routine::process()");
  }

  return false;
}
