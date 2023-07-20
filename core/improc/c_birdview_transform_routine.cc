/*
 * c_birdview_transform_routine.cc
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#include "c_birdview_transform_routine.h"


void c_birdview_transform_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, src_line, "");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, stretch, "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, output_image_size, "");
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

    if( src_size != input_image_size_ ) {
      input_image_size_ = src_size;
      H.release();
    }

    double src_line =
        src_line_ < 0 ? src_size.height * 5 / 8 :
            src_line_;


    cv::Size dst_size =
        output_image_size_;

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
          cv::Point2f(dst_size.width / 2 - stretch_, 0), // top left
          cv::Point2f(dst_size.width / 2 + stretch_, 0), // top right
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

    if( mask.needed() && !mask.empty() ) {

      cv::warpPerspective(mask, mask, H,
          dst_size,
          remap_flags);

      cv::compare(mask, 254, mask,
          cv::CMP_GE);
    }
  }

  return true;
}
