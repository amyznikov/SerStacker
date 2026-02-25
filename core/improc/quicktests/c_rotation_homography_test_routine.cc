/*
 * c_rotation_homography_test_routine.cc
 *
 *  Created on: Aug 17, 2024
 *      Author: amyznikov
 */

#include "c_rotation_homography_test_routine.h"
#include <core/proc/pose.h>


void c_rotation_homography_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "rotation [deg]", ctx(&this_class::_rotation), "rotation angles in [deg]");
  ctlbind(ctls, "focus", ctx(&this_class::_focus), "focus distance in pixels");
  ctlbind(ctls, "center", ctx(&this_class::_center), "Camera principal point");
  ctlbind(ctls, "output_size", ctx(&this_class::_output_size), "output image size");
  ctlbind(ctls, "inverse_remap", ctx(&this_class::_inverse_remap), "inverse_remap");
}

bool c_rotation_homography_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _rotation);
    SERIALIZE_OPTION(settings, save, *this, _focus);
    SERIALIZE_OPTION(settings, save, *this, _center);
    SERIALIZE_OPTION(settings, save, *this, _output_size);
    SERIALIZE_OPTION(settings, save, *this, _inverse_remap);
    return true;
  }
  return false;
}

bool c_rotation_homography_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Size input_size =
      image.size();

  const cv::Size output_size =
      _output_size.empty() ? input_size : _output_size;

  const cv::Matx33f K(
      _focus, 0, _center(0),
      0, _focus,  _center(1),
      0, 0, 1
      );

  const cv::Matx33f H =
      K * build_rotation(_rotation * (float) (CV_PI / 180)) * K.inv();

  const int remap_flags =
      _inverse_remap ? cv::INTER_LINEAR | cv::WARP_INVERSE_MAP :
          cv::INTER_LINEAR;

  if( image.needed() ) {
    cv::warpPerspective(image.getMat(), image,
        H,
        output_size,
        remap_flags,
        cv::BORDER_CONSTANT);
  }

  if( mask.needed() ) {

    if( mask.empty() ) {
      cv::warpPerspective(cv::Mat1b(input_size, 255), mask,
          H,
          output_size,
          remap_flags,
          cv::BORDER_CONSTANT);
    }
    else {
      cv::warpPerspective(mask.getMat(), mask,
          H,
          output_size,
          remap_flags,
          cv::BORDER_CONSTANT);
    }

    cv::compare(mask.getMat(), 252, mask, cv::CMP_GE);
  }

  return true;
}
