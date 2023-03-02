/*
 * c_bilateral_filter_routine.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_bilateral_filter_routine_h__
#define __c_bilateral_filter_routine_h__

#include "c_image_processor.h"

class c_bilateral_filter_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_bilateral_filter_routine,
      "bilateral", "apply cv::bilateralFilter()");

  void set_d(int v)
  {
    d_ = v;
  }

  int d() const
  {
    return d_;
  }

  void set_sigmaColor(double v)
  {
    sigmaColor_ = v;
  }

  double sigmaColor() const
  {
    return sigmaColor_;
  }

  void set_sigmaSpace(double v)
  {
    sigmaSpace_ = v;
  }

  double sigmaSpace() const
  {
    return sigmaSpace_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, d,
        "Diameter of each pixel neighborhood that is used during filtering."
        "If it is non-positive, it is computed from sigmaSpace");

    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigmaColor,
        "Filter sigma in the color space."
        "A larger value of the parameter means that farther colors within the "
        "pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger "
        "areas of semi-equal color");

    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigmaSpace,
        "Filter sigma in the coordinate space. "
        "A larger value of the parameter means that farther pixels will "
        "influence each other as long as their colors are close enough (see sigmaColor). "
        "When d>0, it specifies the neighborhood size regardless of sigmaSpace. "
        "Otherwise, d is proportional to sigmaSpace");

    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderType,
        "Border mode used to extrapolate pixels outside of the image");
  }

  bool serialize(c_config_setting settings, bool save)
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, d);
      SERIALIZE_PROPERTY(settings, save, *this, sigmaColor);
      SERIALIZE_PROPERTY(settings, save, *this, sigmaSpace);
      SERIALIZE_PROPERTY(settings, save, *this, borderType);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    cv::Mat tmp;
    cv::bilateralFilter(image, tmp, d_, sigmaColor_, sigmaSpace_, borderType_);
    image.move(tmp);

    return true;
  }


protected:
  int d_ = 0;
  double sigmaColor_ = 1;
  double sigmaSpace_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
};

#endif /* __c_bilateral_filter_routine_h__ */
