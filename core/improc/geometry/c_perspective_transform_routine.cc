/*
 * c_perspective_transform_routine.cc
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#include "c_perspective_transform_routine.h"
//#include <core/proc/birds-eye-view.h>
#include <core/proc/pose.h>

void c_perspective_transform_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, src_point0, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, dst_point0, "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, src_point1, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, dst_point1, "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, src_point2, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, dst_point2, "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, src_point3, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, dst_point3, "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, output_image_size, "");
}

bool c_perspective_transform_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, src_point0);
    SERIALIZE_PROPERTY(settings, save, *this, src_point1);
    SERIALIZE_PROPERTY(settings, save, *this, src_point2);
    SERIALIZE_PROPERTY(settings, save, *this, src_point3);

    SERIALIZE_PROPERTY(settings, save, *this, dst_point0);
    SERIALIZE_PROPERTY(settings, save, *this, dst_point1);
    SERIALIZE_PROPERTY(settings, save, *this, dst_point2);
    SERIALIZE_PROPERTY(settings, save, *this, dst_point3);

    SERIALIZE_PROPERTY(settings, save, *this, output_image_size);
    return true;
  }
  return false;
}

bool c_perspective_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    cv::Size outImageSize =
        output_image_size_.empty() ? image.size() :
            output_image_size_;

    if( H.empty() ) {

      H =
          cv::getPerspectiveTransform(src_pts,
              dst_pts);

      if( H.empty() ) {
        CF_ERROR("getPerspectiveTransform() fails");
      }
    }

    const int remap_flags =
        cv::INTER_LINEAR;

    cv::warpPerspective(image, image, H,
        outImageSize,
        remap_flags);

    if( mask.needed() && !mask.empty() ) {

      cv::warpPerspective(mask, mask, H,
          outImageSize,
          remap_flags);

      cv::compare(mask, 254, mask,
          cv::CMP_GE);
    }
  }

  return true;
}
