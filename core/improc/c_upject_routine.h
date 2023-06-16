/*
 * c_upject_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_upject_routine_h__
#define __c_upject_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/downstrike.h>

class c_upject_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_upject_routine,
      "upject",
      "2x upsampling step by injecting EVEN or UNEVEN rows and columns");

  enum UpjectMode {
    UpjectUneven,
    UpjectEven,
  };

  enum FillMode {
    FillZero,
    FillAvg,
  };

  void set_mode(UpjectMode v)
  {
    mode_ = v;
  }

  UpjectMode mode() const
  {
    return mode_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, mode, "Which rows and columns to reject");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override
  {
    cv::Mat img;
    cv::Mat zmask;

    switch (mode_) {
      case UpjectUneven:
        upject_uneven(image.getMat(), img, dstSize_, &zmask);
        image.move(img);
        if( mask.needed() && !mask.empty() ) {
          downstrike_uneven(mask.getMat(), img);
          mask.move(img);
        }
        break;
      case UpjectEven:
        upject_even(image.getMat(), img, dstSize_, &zmask);
        image.move(img);
        if( mask.needed() && !mask.empty() ) {
          downstrike_even(mask.getMat(), img);
          mask.move(img);
        }
        break;
      default:
        CF_ERROR("Invalid downstrike mode specified");
        return false;
    }
    return true;
  }

protected:
  UpjectMode mode_ = UpjectUneven;
  cv::Size dstSize_ = cv::Size (-1,-1);
};

#endif /* __c_upject_routine_h__ */
