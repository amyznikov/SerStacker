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

  enum DisplayType {
    DisplayLaplacian,
    DisplayMean,
  };

  void set_display_type(DisplayType v)
  {
    display_type_ = v;
  }

  DisplayType display_type() const
  {
    return display_type_;
  }

  void set_display_level(int v)
  {
    display_level_ = v;
  }

  int display_level() const
  {
    return display_level_;
  }

  void set_absdiff(bool v)
  {
    absdiff_ = v;
  }

  bool absdiff() const
  {
    return absdiff_;
  }

  void set_min_image_size(int v)
  {
    minimum_image_size_ = v;
  }

  int min_image_size() const
  {
    return minimum_image_size_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, display_type, "Specify display type");
    BIND_PCTRL(ctls, display_level, "Specify display pyramid layer");
    BIND_PCTRL(ctls, absdiff, "Set true to show laplacian absolute value");
    BIND_PCTRL(ctls, min_image_size, "Specify minimum image size");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, display_type);
      SERIALIZE_PROPERTY(settings, save, *this, display_level);
      SERIALIZE_PROPERTY(settings, save, *this, absdiff);
      SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    build_laplacian_pyramid(image,
        lp_,
        std::max(1, minimum_image_size_),
        borderType_,
        &mp_);

    if( lp_.size() < 1 ) {
      CF_ERROR("build_laplacian_pyramid() fails");
      return false;
    }

    switch (display_type_) {
      case DisplayMean: {
        const int display_level =
            std::max(0, std::min(display_level_,
                (int) mp_.size() - 1));

        mp_[display_level].copyTo(image);
        break;
      }
      case DisplayLaplacian:
        default: {
        const int display_level =
            std::max(0, std::min(display_level_,
                (int) lp_.size() - 1));

        if ( absdiff_ ) {
          cv::absdiff(lp_[display_level], cv::Scalar::all(0), image);
        }
        else {
          lp_[display_level].copyTo(image);
        }
        break;
      }
    }


    if ( mask.needed() && !mask.empty() ) {
      mask.release();
    }

    return true;
  }


protected:
  DisplayType display_type_ = DisplayLaplacian;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
  int minimum_image_size_ = 4;
  int display_level_ = 1;
  bool absdiff_ = false;
  std::vector<cv::Mat> lp_, mp_;
};

#endif /* __c_laplacian_pyramid_routine_h__ */
