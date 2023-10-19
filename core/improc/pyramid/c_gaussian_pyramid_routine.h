/*
 * c_gaussian_pyramid_routine.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyrdown_routine_h__
#define __c_pyrdown_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unsharp_mask.h>

class c_gaussian_pyramid_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_pyramid_routine, "gaussian_pyramid",
      "Calls <strong>cv::pyrDown()</strong> or <strong>cv::pyrUp()</strong> on image");

  void set_count(int v)
  {
    count_ = v;
  }

  int count() const
  {
    return count_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }

  void set_usharp_sigma(double v)
  {
    usharp_sigma_ = v;
  }

  double usharp_sigma() const
  {
    return usharp_sigma_;
  }

  void set_usharp_alpha(double v)
  {
    usharp_alpha_ = v;
  }

  double usharp_alpha() const
  {
    return usharp_alpha_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, count, "count of times for pyDown (negative value for pyrUp instead)");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderType, "enum cv::BorderTypes");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_sigma, "Optional unsharp mask sigma before downscaling (set 0 to disable)");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_alpha, "Optional unsharp mask alpha before downscaling (set 0 to disable)");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, count);
      SERIALIZE_PROPERTY(settings, save, *this, borderType);
      SERIALIZE_PROPERTY(settings, save, *this, usharp_sigma);
      SERIALIZE_PROPERTY(settings, save, *this, usharp_alpha);
      return true;

    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if( count_ > 0 ) {

      const bool trivialMask =
          mask.empty() || cv::countNonZero(mask) == mask.size().area();

      for( int i = 0; i < count_ && std::min(image.cols(), image.rows()) > 3; ++i ) {

        if( usharp_sigma_ > 0 && usharp_alpha_ > 0 ) {
          unsharp_mask(image.getMat(), mask.getMat(),
              image,
              usharp_sigma_,
              usharp_alpha_);
        }

        cv::pyrDown(image.getMat(), image, cv::Size(), borderType_);

        if( !trivialMask ) {
          cv::pyrDown(mask.getMat(), mask, cv::Size(), borderType_);
        }
      }

      if( !mask.empty() ) {

        if( !trivialMask ) {
          cv::compare(mask, 255, mask, cv::CMP_GE);
        }
        else {
          cv::resize(mask, mask, image.size(), 0, 0, cv::INTER_NEAREST);
        }
      }

    }
    else if( count_ < 0 ) {

      const bool trivialMask =
          mask.empty() || cv::countNonZero(mask) == mask.size().area();

      for( int i = 0; i < -count_ && std::max(image.cols(), image.rows()) < 16000; ++i ) {

        if( usharp_sigma_ > 0 && usharp_alpha_ > 0 ) {
          unsharp_mask(image.getMat(), mask.getMat(),
              image,
              usharp_sigma_,
              usharp_alpha_);
        }

        cv::pyrUp(image.getMat(), image, cv::Size(), borderType_);

        if( !trivialMask ) {
          cv::pyrUp(mask.getMat(), mask, cv::Size(), borderType_);
        }

      }

      if( !mask.empty() ) {

        if( !trivialMask ) {
          cv::compare(mask.getMat(), 255, mask, cv::CMP_GE);
        }
        else {
          cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_NEAREST);
        }
      }
    }

    return true;
  }

protected:
  int count_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
  double usharp_sigma_ = 1.5;
  double usharp_alpha_ = 0.0;
};

#endif /* __c_pyrdown_routine_h__ */
