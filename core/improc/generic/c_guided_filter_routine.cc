/*
 * c_guided_filter_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_guided_filter_routine.h"
#include <opencv2/ximgproc.hpp>

void c_guided_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "radius", ctx(&this_class::radius),
      "Radius of Guided Filter");

  ctlbind(ctls, "eps", ctx(&this_class::eps),
      "Regularization term of Guided Filter.\n"
      "The eps^2 is similar to the sigma in the color space in bilateralFilter");

  ctlbind(ctls, "scale", ctx(&this_class::scale),
      "Subsample factor of Fast Guided Filter.\n"
      " Use a scale less than 1 to speeds up computation with almost no visible degradation.\n"
      " (e.g. scale==0.5 shrinks the image by 2x inside the filter)");

  ctlbind(ctls, "mkgr", ctx(&this_class::mkgr),
      "Apply medianBlur(src, guide, 2*mkgr+1) to create the guide image.\n");

  ctlbind(ctls, "ddepth", ctx(&this_class::ddepth),
      "Optional depth of the output image");
}

bool c_guided_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, ddepth);
    SERIALIZE_OPTION(settings, save, *this, radius);
    SERIALIZE_OPTION(settings, save, *this, eps);
    SERIALIZE_OPTION(settings, save, *this, scale);
    SERIALIZE_OPTION(settings, save, *this, mkgr);
    return true;
  }
  return false;
}

bool c_guided_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat guide;
  const cv::Mat src = image.getMat();

  if ( mkgr < 1 ) {
    guide = src;
  }
  else {
    cv::medianBlur(src, guide, 2 * mkgr + 1);
  }

  cv::ximgproc::guidedFilter(guide, src, image, radius, eps, ddepth, scale);
  return true;
}
