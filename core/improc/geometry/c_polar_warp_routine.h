/*
 * c_polar_warp_routine.h
 *
 *  Created on: Aug 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_polar_warp_routine_h__
#define __c_polar_warp_routine_h__

#include <core/improc/c_image_processor.h>

class c_polar_warp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_polar_warp_routine,
      "polar_warp", "Apply cv::warpPolar() to image");

  enum INTERPOLATION_MODE
  {
    INTER_NEAREST = cv::INTER_NEAREST,
    INTER_LINEAR = cv::INTER_LINEAR,
    INTER_CUBIC = cv::INTER_CUBIC,
    INTER_AREA = cv::INTER_AREA,
    INTER_LANCZOS4 = cv::INTER_LANCZOS4,
    INTER_LINEAR_EXACT = cv::INTER_LINEAR_EXACT,
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
    INTER_NEAREST_EXACT = cv::INTER_NEAREST_EXACT,
#endif
  };

  enum WARP_MODE
  {
    WARP_POLAR_LINEAR = cv::WARP_POLAR_LINEAR,
    WARP_POLAR_LOG = cv::WARP_POLAR_LOG
  };


  const cv::Point2f & center() const
  {
    return center_;
  }

  void set_center(const cv::Point2f & v)
  {
    center_ = v;
  }

  const cv::Size & dsize() const
  {
    return dsize_;
  }

  void set_dsize(const cv::Size & v)
  {
    dsize_ = v;
  }


  double maxRadius() const
  {
    return maxRadius_;
  }

  void set_maxRadius(double v)
  {
    maxRadius_ = v;
  }

  INTERPOLATION_MODE interpolation_mode() const
  {
    return interpolation_mode_;
  }

  void set_interpolation_mode(INTERPOLATION_MODE v)
  {
    interpolation_mode_ = v;
  }

  WARP_MODE warp_mode() const
  {
    return warp_mode_;
  }

  void set_warp_mode(WARP_MODE v)
  {
    warp_mode_ = v;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, center, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, dsize, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, maxRadius, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, interpolation_mode, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, warp_mode, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, center);
      SERIALIZE_PROPERTY(settings, save, *this, dsize);
      SERIALIZE_PROPERTY(settings, save, *this, maxRadius);
      SERIALIZE_PROPERTY(settings, save, *this, interpolation_mode);
      SERIALIZE_PROPERTY(settings, save, *this, warp_mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if( !image.empty() || !mask.empty()) {

      const cv::Size src_size =
          mask.empty() ? image.size() :
              mask.size();

      cv::Mat1b msk;
      cv::warpPolar(cv::Mat1b(src_size, (uint8_t)255), msk,
          dsize_,
          center_,
          maxRadius_,
          cv::INTER_LINEAR | warp_mode_);
      cv::compare(msk, 254, msk, cv::CMP_LT);


      if( !image.empty() ) {
        cv::warpPolar(image.getMat(), image,
            dsize_,
            center_,
            maxRadius_,
            interpolation_mode_ | warp_mode_);
        image.setTo(0, msk);
      }

      if ( mask.needed() ) {
        if( mask.empty() ) {
          cv::bitwise_not(msk, mask);
        }
        else {
          cv::warpPolar(mask.getMat(), mask,
              dsize_,
              center_,
              maxRadius_,
              cv::INTER_LINEAR | warp_mode_);
          cv::compare(mask, 254, mask, cv::CMP_GE);
          mask.setTo(0, msk);
        }
      }
    }

    return true;
  }

protected:
  cv::Point2f center_;
  cv::Size dsize_ = cv::Size(100, 100);
  double maxRadius_ = 100;
  INTERPOLATION_MODE interpolation_mode_ = INTER_LINEAR;
  WARP_MODE warp_mode_ = WARP_POLAR_LINEAR;

};

#endif /* __c_polar_warp_routine_h__ */
