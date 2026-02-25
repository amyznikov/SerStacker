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

void c_selective_search_segmentation_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "strategy", ctx, &this_class::strategy, &this_class::set_strategy, "");
  ctlbind(ctls, "base_k", ctx, &this_class::base_k, &this_class::set_base_k, "");
  ctlbind(ctls, "inc_k", ctx, &this_class::inc_k, &this_class::set_inc_k, "");
  ctlbind(ctls, "sigma", ctx, &this_class::sigma, &this_class::set_sigma, "");
  ctlbind(ctls, "max_display_rects", ctx, &this_class::max_display_rects, &this_class::set_max_display_rects, "");
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

    if( !_ss ) {
      _ss = cv::ximgproc::segmentation::createSelectiveSearchSegmentation();
    }

    try {
      _ss->setBaseImage(image.getMat());

      switch (_strategy) {
        case Fast:
          _ss->switchToSelectiveSearchFast(_base_k, _inc_k, _sigma);
          break;
        case Quality:
          _ss->switchToSelectiveSearchQuality(_base_k, _inc_k, _sigma);
          break;
        case Single:
          _ss->switchToSingleStrategy(_base_k, _sigma);
          break;
      }

      _rects.clear();
      _ss->process(_rects);

      CF_DEBUG("%zu rects proposed", _rects.size());
    }
    catch (const std::exception &e) {
      CF_ERROR("ss->process() fails: %s", e.what());
      return false;
    }


    const int max_display_rects =
        std::min(_max_display_rects,
            (int) (_rects.size()));

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
      cv::rectangle(image, _rects[i], color);
    }
  }

  return true;
}
