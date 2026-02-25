/*
 * c_unkanala_remap_routine.cc
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#include "c_unkanala_remap_routine.h"


void c_unkanala_remap_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "image_size", ctx, &this_class::image_size, &this_class::set_image_size, "");
  ctlbind(ctls, "focal_length_x", ctx, &this_class::focal_length_x, &this_class::set_focal_length_x, "");
  ctlbind(ctls, "focal_length_y", ctx, &this_class::focal_length_y, &this_class::set_focal_length_y, "");
  ctlbind(ctls, "principal_point_x", ctx, &this_class::principal_point_x, &this_class::set_principal_point_x, "");
  ctlbind(ctls, "principal_point_y", ctx, &this_class::principal_point_y, &this_class::set_principal_point_y, "");
  ctlbind(ctls, "distortion_coefficients", ctx, &this_class::distortion_coefficients, &this_class::set_distortion_coefficients, "must be 4 items");
}

bool c_unkanala_remap_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, image_size);
    SERIALIZE_PROPERTY(settings, save, *this, focal_length_x);
    SERIALIZE_PROPERTY(settings, save, *this, focal_length_y);
    SERIALIZE_PROPERTY(settings, save, *this, principal_point_x);
    SERIALIZE_PROPERTY(settings, save, *this, principal_point_y);
    SERIALIZE_PROPERTY(settings, save, *this, distortion_coefficients);
    return true;
  }
  return false;
}

bool c_unkanala_remap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _remap.empty() ) {

    if ( !create_unkanala_remap(_intrinsics, _remap) ) {
      CF_ERROR("create_unkanala_remap() fails");
      return  false;
    }
  }

  if( image.needed() ) {
    cv::remap(image.getMat(), image,
        _remap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
  }

  if( mask.needed() ) {

    if ( !mask.empty() ) {
      cv::remap(mask.getMat(), mask,
          _remap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }
    else {
      cv::remap(cv::Mat1b(image.size(), 255), mask,
          _remap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }

    cv::compare(mask.getMat(), 250, mask, cv::CMP_GT);
  }


  return true;
}

