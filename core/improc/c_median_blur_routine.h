/*
 * c_image_processor_routine.h
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_blur_routine_h__
#define __c_median_blur_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/ximgproc.hpp>

class c_median_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_blur_routine,
      "medianBlur",
      "Calls <strong>cv::medianBlur()</strong>");

  void set_radius(int v)
  {
    radius_ = v;
  }

  int radius() const
  {
    return radius_;
  }

  void set_iterations(int v)
  {
    iterations_ = v;
  }

  int iterations() const
  {
    return iterations_;
  }


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, radius, "Filter radius [px]");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, iterations, "Number of iterations");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, radius);
      SERIALIZE_PROPERTY(settings, save, *this, iterations);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    const int r = image.depth() < CV_32F ? radius_ : 2;
    const int ksize = 2 * radius_ + 1;

    for( int i = 0; i < iterations_; ++i ) {
      cv::medianBlur(image, image, ksize);
    }

    return true;
  }

protected:
  int radius_ = 1;
  int iterations_ = 1;
};

#endif /* __c_median_blur_routine_h__ */
