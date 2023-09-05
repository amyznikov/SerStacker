/*
 * c_morph_gradient_pyramid_routine.cc
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#include "c_morph_gradient_pyramid_routine.h"
#include <core/proc/downstrike.h>

static void build_morph_gradient_pyramid(cv::InputArray image, cv::InputArray mask,
    std::vector<cv::Mat> & layers,
    int minimum_image_size = 16)
{
  static const cv::Mat1b SE(3, 3, 255);

  layers.clear();
  layers.reserve(10);

  layers.emplace_back();

  cv::morphologyEx(image, layers.back(),
      cv::MORPH_GRADIENT,
      SE,
      cv::Point(-1, -1),
      1,
      cv::BORDER_REPLICATE);

  if( !mask.empty() ) {

    cv::Mat m;

    cv::morphologyEx(mask, m,
        cv::MORPH_ERODE,
        SE,
        cv::Point(-1, -1),
        1,
        cv::BORDER_CONSTANT);

    layers.back().setTo(0, ~m);
  }


  while (42) {

    const cv::Size current_size =
        layers.back().size();

    const cv::Size next_size =
        cv::Size((current_size.width + 1) / 2, (current_size.height + 1) / 2);

    if( std::min(next_size.width, next_size.height) <= minimum_image_size ) {
      break;
    }

    layers.emplace_back();

    cv::morphologyEx(layers[layers.size() - 2], layers.back(),
        cv::MORPH_DILATE,
        SE,
        cv::Point(-1, -1),
        1,
        cv::BORDER_REPLICATE);

    downstrike_uneven(layers.back(),
        layers.back());
  }

}


void c_morph_gradient_pyramid_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  base::get_parameters(ctls);
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_image_size, "Specify minimum image size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_pos, "Specify display node position");
}

bool c_morph_gradient_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, display_pos);
    return true;
  }
  return false;
}

bool c_morph_gradient_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int display_pos = -1;

  if ( image.needed() && !image.empty() ) {

    build_morph_gradient_pyramid(image,
        ignore_mask_ ? cv::noArray() : mask,
        pyramid_,
        minimum_image_size_);

    display_pos =
        std::max(0, std::min(display_pos_,
            (int) pyramid_.size() - 1));

    pyramid_[display_pos].copyTo(image);
  }

  if ( mask.needed() && !mask.empty() && display_pos > 0 ) {
    cv::resize(mask.getMat(), mask, pyramid_[display_pos].size(), 0, 0, cv::INTER_AREA);
    cv::compare(mask, 250, mask, cv::CMP_GE);
  }

  return true;
}
