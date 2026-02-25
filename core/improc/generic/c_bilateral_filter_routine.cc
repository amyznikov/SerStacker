/*
 * c_bilateral_filter_routine.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "c_bilateral_filter_routine.h"

void c_bilateral_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "d", ctx(&this_class::_d),
      "Diameter of each pixel neighborhood that is used during filtering.\n"
          "If it is non-positive, it is computed from sigmaSpace");

  ctlbind(ctls, "sigmaColor", ctx(&this_class::_sigmaColor),
      "Filter sigma in the color space.\n"
          "A larger value of the parameter means that farther colors within the "
          "pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger "
          "areas of semi-equal color");

  ctlbind(ctls, "sigmaSpace", ctx(&this_class::_sigmaSpace),
      "Filter sigma in the coordinate space.\n"
          "A larger value of the parameter means that farther pixels will "
          "influence each other as long as their colors are close enough (see sigmaColor). "
          "When d>0, it specifies the neighborhood size regardless of sigmaSpace. "
          "Otherwise, d is proportional to sigmaSpace");

  ctlbind(ctls, "borderType", ctx(&this_class::_borderType),
      "Border mode used to extrapolate pixels outside of the image");
}

bool c_bilateral_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _d);
    SERIALIZE_OPTION(settings, save, *this, _sigmaColor);
    SERIALIZE_OPTION(settings, save, *this, _sigmaSpace);
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    return true;
  }
  return false;
}

bool c_bilateral_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat tmp;
  cv::bilateralFilter(image, tmp, _d, _sigmaColor, _sigmaSpace, _borderType);
  image.move(tmp);

  return true;
}

