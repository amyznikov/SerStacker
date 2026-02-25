/*
 * c_birdview_transform_routine.cc
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#include "c_birdview_transform_routine.h"

void c_birdview_transform_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "src_line", ctx, &this_class::src_line, &this_class::set_src_line, "");
   ctlbind(ctls, "stretch", ctx, &this_class::stretch, &this_class::set_stretch, "");
   ctlbind(ctls, "output_size", ctx, &this_class::output_image_size, &this_class::set_output_image_size, "");
}

bool c_birdview_transform_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, src_line);
    SERIALIZE_PROPERTY(settings, save, *this, stretch);
    SERIALIZE_PROPERTY(settings, save, *this, output_image_size);
    return true;
  }
  return false;
}

bool c_birdview_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    const cv::Size src_size =
        image.size();

    if( src_size != _input_image_size ) {
      _input_image_size = src_size;
      H.release();
    }

    double src_line =
        _src_line < 0 ? src_size.height * 5 / 8 :
            _src_line;


    cv::Size dst_size =
        _output_image_size;

    if ( dst_size.width < 1 ) {
      dst_size.width = 3 * src_size.width/2;
    }

    if ( dst_size.height < 1 ) {
      dst_size.height = 4 * src_size.height;
    }

    if( H.empty() ) {

      const cv::Point2f src_points[4] = {
          cv::Point2f(src_size.width/2 - 1, src_line), // top left
          cv::Point2f(src_size.width/2 + 1, src_line), // top right
          cv::Point2f(src_size.width-1, src_size.height-1), // bottom right
          cv::Point2f(0, src_size.height-1), // bottom left
      };

      const cv::Point2f dst_points[4] = {
          cv::Point2f(dst_size.width / 2 - _stretch, 0), // top left
          cv::Point2f(dst_size.width / 2 + _stretch, 0), // top right
          cv::Point2f((dst_size.width + src_size.width) / 2, dst_size.height - 1), // bottom right
          cv::Point2f((dst_size.width - src_size.width) / 2, dst_size.height - 1), // bottom left
      };

      if( (H = cv::getPerspectiveTransform(src_points, dst_points)).empty() ) {
        CF_ERROR("getPerspectiveTransform() fails");
        return false;
      }
    }

    const int remap_flags =
        cv::INTER_LINEAR;

    cv::warpPerspective(image, image, H,
        dst_size,
        remap_flags);

    if( mask.needed() ) {

      if ( !mask.empty() ) {
        cv::warpPerspective(mask, mask, H,
            dst_size,
            remap_flags,
            cv::BORDER_CONSTANT);
      }
      else {
        cv::warpPerspective(cv::Mat1b(src_size, 255), mask, H,
            dst_size,
            remap_flags,
            cv::BORDER_CONSTANT);
      }

      cv::compare(mask, 254, mask,
          cv::CMP_GE);
    }
  }

  return true;
}
