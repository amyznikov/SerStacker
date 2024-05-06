/*
 * c_amelp_pyramid_routine.cc
 *
 *  Created on: May 6, 2024
 *      Author: amyznikov
 */

#include "c_amelp_pyramid_routine.h"


static void build_amelp_pyramid(cv::InputArray image,
    std::vector<cv::Mat> & layers,
    int max_level,
    double scale_factor )
{
  const int num_levels =
      max_level + 1;

  layers.clear();
  layers.reserve(num_levels);

  cv::Mat tmp, simg;

  double scale = 1;

  cv::resize(image, simg, cv::Size(image.cols() * 2, image.rows() * 2),
      0, 0,
      cv::INTER_CUBIC);

  for ( int level = 0; level < num_levels; ++level, scale *= scale_factor ) {

    layers.emplace_back();

    cv::resize(image, layers.back(), cv::Size(),
        scale, scale,
        cv::INTER_AREA);

    cv::resize(layers.back(), tmp, simg.size(),
          0, 0,
          cv::INTER_CUBIC);

    cv::absdiff(simg, tmp, tmp);

    cv::resize(tmp, tmp, layers.back().size(),
          0, 0,
          cv::INTER_AREA);

    cv::add(tmp, layers.back(), layers.back());
  }
}


void c_amelp_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, scale_factor,  "scale_factor", "Specify scale_factor");
  BIND_SPINBOX_CTRL(ctls, max_level, 0, 32, 1,  "max_level", "Specify max pyramid level");
  BIND_SPINBOX_CTRL(ctls, display_pos, 0, 32, 1, "display level", "Specify display pyramid level");
}

bool c_amelp_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, scale_factor);
    SERIALIZE_PROPERTY(settings, save, *this, max_level);
    SERIALIZE_PROPERTY(settings, save, *this, display_pos);
    return true;
  }
  return false;
}

bool c_amelp_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int display_pos = -1;

  if ( image.needed() && !image.empty() ) {

    build_amelp_pyramid(image,
        pyramid_,
        max_level_,
        scale_factor_);

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

