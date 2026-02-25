/*
 * c_perspective_transform_routine.cc
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#include "c_perspective_transform_routine.h"
#include <core/proc/pose.h>

void c_perspective_transform_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "srcp0", ctx, &this_class::src_point0, &this_class::set_src_point0, "");
   ctlbind(ctls, "srcp1", ctx, &this_class::src_point1, &this_class::set_src_point1, "");
   ctlbind(ctls, "srcp2", ctx, &this_class::src_point2, &this_class::set_src_point2, "");
   ctlbind(ctls, "srcp3", ctx, &this_class::src_point3, &this_class::set_src_point3, "");
   ctlbind(ctls, "dstp0", ctx, &this_class::dst_point0, &this_class::set_dst_point0, "");
   ctlbind(ctls, "dstp1", ctx, &this_class::dst_point1, &this_class::set_dst_point1, "");
   ctlbind(ctls, "dstp2", ctx, &this_class::dst_point2, &this_class::set_dst_point2, "");
   ctlbind(ctls, "dstp3", ctx, &this_class::dst_point3, &this_class::set_dst_point3, "");
   ctlbind(ctls, "output_image_size", ctx, &this_class::output_image_size, &this_class::set_output_image_size, "");
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
        _output_image_size.empty() ? image.size() :
            _output_image_size;

    if( H.empty() ) {

      const cv::Point2f  src_pts[4] = {_srcp0, _srcp1, _srcp2, _srcp3};
      const cv::Point2f  dst_pts[4] = {_dstp0, _dstp1, _dstp2, _dstp3};

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
