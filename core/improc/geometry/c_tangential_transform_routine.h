/*
 * c_tangential_transform_routine.h
 *
 *  Created on: Jul 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_tangential_transform_routine_h__
#define __c_tangential_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_tangential_transform_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_tangential_transform_routine,
      "tangential_transform", "Apply tangential transform to image");

  void set_focus(double v)
  {
    focus_ = v;
    rmap_.release();
  }

  double focus() const
  {
    return focus_;
  }

  void set_center(const cv::Point2d & v)
  {
    center_ = v;
    rmap_.release();
  }

  const cv::Point2d & center() const
  {
    return center_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, focus, "focal distance in pixels");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, center, "optical center in pixels");
  }

  bool serialize(c_config_setting settings, bool save)
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, focus);
      SERIALIZE_PROPERTY(settings, save, *this, center);
      return true;
    }
    return false;
  }


  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    if( !image.empty() || !mask.empty() ) {

      const cv::Size image_size =
          image.empty() ? mask.size() : image.size();

      if ( rmap_.empty() || previous_image_size != image_size ) {

        if ( center_.x == -1 && center_.y == -1 ) {
          center_.x = image_size.width / 2;
          center_.y = image_size.height / 2;
        }

        double al = atan2(0 - center_.x, focus_);
        double at = atan2(0 - center_.y, focus_);
        double ar = atan2(image_size.width - center_.x, focus_);
        double ab = atan2(image_size.height - center_.y, focus_);

        int l = cvRound(al * focus_);
        int t = cvRound(at * focus_);
        int r = cvRound(ar * focus_);
        int b = cvRound(ab * focus_);

        const cv::Size remap_size(r - l, b - t);

        rmap_.create(remap_size);

        for ( int y = 0; y < remap_size.height; ++y ) {
          for ( int x = 0; x < remap_size.width; ++x ) {

            double ax = (x + l) / focus_;
            double ay = (y + t) / focus_;

            rmap_[y][x][0] = tan(ax) * focus_ + center_.x;
            rmap_[y][x][1] = tan(ay) * focus_ + center_.y;
          }
        }
      }

      if( !image.empty() ) {
        cv::remap(image, image, rmap_, cv::noArray(), cv::INTER_LINEAR);
      }

      if( !mask.empty() ) {
        cv::remap(mask, mask, rmap_, cv::noArray(), cv::INTER_LINEAR);
        cv::compare(mask, 245, mask, cv::CMP_GE);
      }

      previous_image_size = image_size;

    }

    return true;
  }

protected:
  double focus_ = 1000;
  cv::Point2d center_ = cv::Point2d(-1, -1);
  cv::Mat2f rmap_;
  cv::Size previous_image_size;
};

#endif /* __c_tangential_transform_routine_h__ */
