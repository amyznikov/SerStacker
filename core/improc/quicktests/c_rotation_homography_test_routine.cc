/*
 * c_rotation_homography_test_routine.cc
 *
 *  Created on: Aug 17, 2024
 *      Author: amyznikov
 */

#include "c_rotation_homography_test_routine.h"
#include <core/proc/pose.h>



void c_rotation_homography_test_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, rotation, "rotation angles in [deg]");
  BIND_PCTRL(ctls, focus, "focus distance in pixels");
  BIND_PCTRL(ctls, center, "Camera principal point");
  BIND_PCTRL(ctls, output_size, "output image size");
  BIND_PCTRL(ctls, inverse_remap, "inverse_remap");


}

bool c_rotation_homography_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, rotation);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    SERIALIZE_PROPERTY(settings, save, *this, focus);
    SERIALIZE_PROPERTY(settings, save, *this, output_size);
    SERIALIZE_PROPERTY(settings, save, *this, inverse_remap);

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
      _F, 0, _C(0),
      0, _F,  _C(1),
      0, 0, 1
      );

  const cv::Matx33f H =
      K * build_rotation(_A * (float) (CV_PI / 180)) * K.inv();

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
