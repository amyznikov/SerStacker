/*
 * c_laplacian_pyramid_routine.h
 *
 *  Created on: May 30, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_pyramid_routine_h__
#define __c_laplacian_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/laplacian_pyramid.h>

class c_laplacian_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_laplacian_pyramid_routine,
      "laplacian_pyramid", "Display Laplacian Pyramid layers");

  void set_min_image_size(int v)
  {
    minimum_image_size_ = v;
  }

  int min_image_size() const
  {
    return minimum_image_size_;
  }

  void set_display_level(int v)
  {
    display_level_ = v;
  }

  int display_level() const
  {
    return display_level_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, min_image_size, "Specify minimum image size");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display_level, "Specify display pyramid layer");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
      SERIALIZE_PROPERTY(settings, save, *this, display_level);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    int minimum_image_size = 4;

    pyramid_.clear();

    build_laplacian_pyramid(image,
        pyramid_,
        std::max(1, minimum_image_size_));

    if( pyramid_.size() < 1 ) {
      CF_ERROR("build_laplacian_pyramid() fails");
      return false;
    }

    const int display_level =
        std::max(0, std::min(display_level_,
            (int) pyramid_.size() - 1));

    pyramid_[display_level].copyTo(image);

    if ( mask.needed() && !mask.empty() ) {
      mask.release();
    }

    return true;
  }


protected:
  int minimum_image_size_ = 4;
  int display_level_ = 1;
  std::vector<cv::Mat> pyramid_;
};

#endif /* __c_laplacian_pyramid_routine_h__ */
