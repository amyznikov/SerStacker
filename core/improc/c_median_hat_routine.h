/*
 * c_median_hat_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_hat_routine_h__
#define __c_median_hat_routine_h__

#include <core/improc/c_image_processor.h>

class c_median_hat_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_hat_routine,
      "median_hat",
      "Difference between input image and it's median blur");

  void set_radius(int v)
  {
    radius_ = v;
  }

  int radius() const
  {
    return radius_;
  }

  void set_absdiff(bool v)
  {
    absdiff_ = v;
  }

  bool absdiff() const
  {
    return absdiff_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, radius, "Filter radius [px]");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, absdiff, "Use cv::absdiff() instead of cv::subtract()");

  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, radius);
      SERIALIZE_PROPERTY(settings, save, *this, absdiff);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    const int r = image.depth() < CV_32F ? radius_ : 2;
    const int ksize = 2 * radius_ + 1;

    cv::Mat m;

    cv::medianBlur(image, m, ksize);
    if ( absdiff_ ) {
      cv::absdiff(image, m, image);
    }
    else {
      cv::subtract(image, m, image);
    }

    return true;
  }

protected:
  int radius_ = 1;
  bool absdiff_ = false;
};

#endif /* __c_median_hat_routine_h__ */
