/*
 * c_selective_search_segmentation_routine.cc
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#include "c_selective_search_segmentation_routine.h"

template<>
const c_enum_member* members_of<c_selective_search_segmentation_routine::Strategy>()
{
  static const c_enum_member members[] = {
      { c_selective_search_segmentation_routine::Fast, "Fast", "" },
      { c_selective_search_segmentation_routine::Quality, "Quality", "" },
      { c_selective_search_segmentation_routine::Single, "Single", "Single" },
      { c_selective_search_segmentation_routine::Fast }
  };

  return members;
}

void c_selective_search_segmentation_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, strategy, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, base_k, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, inc_k, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_display_rects, "");
}

bool c_selective_search_segmentation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, strategy);
    SERIALIZE_PROPERTY(settings, save, *this, base_k);
    SERIALIZE_PROPERTY(settings, save, *this, inc_k);
    SERIALIZE_PROPERTY(settings, save, *this, sigma);
    SERIALIZE_PROPERTY(settings, save, *this, max_display_rects);
    return true;
  }
  return false;
}

bool c_selective_search_segmentation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.needed() && !image.empty() ) {

    if( !ss_ ) {
      ss_ = cv::ximgproc::segmentation::createSelectiveSearchSegmentation();
    }

    try {
      ss_->setBaseImage(image.getMat());

      switch (strategy_) {
        case Fast:
          ss_->switchToSelectiveSearchFast(base_k_, inc_k_, sigma_);
          break;
        case Quality:
          ss_->switchToSelectiveSearchQuality(base_k_, inc_k_, sigma_);
          break;
        case Single:
          ss_->switchToSingleStrategy(base_k_, sigma_);
          break;
      }

      rects_.clear();
      ss_->process(rects_);

      CF_DEBUG("%zu rects proposed", rects_.size());
    }
    catch (const std::exception &e) {
      CF_ERROR("ss->process() fails: %s", e.what());
      return false;
    }


    const int max_display_rects =
        std::min(max_display_rects_,
            (int) (rects_.size()));

    double color_scale = 1.0;
    switch (image.depth()) {
      case CV_16U:
      case CV_16S:
        color_scale = 255;
        break;
      case CV_32S:
        color_scale = 65535;
        break;
      case CV_32F:
      case CV_64F:
        color_scale = 1./255;
        break;
    }


    if ( image.channels() == 1 ) {
      cv::cvtColor(image.getMat(), image, cv::COLOR_GRAY2BGR);
    }

    for( int i = 0; i < max_display_rects; ++i ) {
      cv::Scalar color(rand() % 255 + 32, rand() % 255 + 32, rand() % 255 + 32);
      cv::rectangle(image, rects_[i], color);
    }
  }

  return true;
}
