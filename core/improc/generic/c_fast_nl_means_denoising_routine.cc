/*
 * c_fast_nl_means_denoising_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_fast_nl_means_denoising_routine.h"

void c_fast_nl_means_denoising_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "h", ctx(&this_class::h),
      "Parameter regulating filter strength for luminance component.\n"
      " Bigger h value perfectly removes noise but also removes image details,\n"
      " smaller h value preserves details but also preserves some noise");

  ctlbind(ctls, "hColor", ctx(&this_class::hColor),
      "The same as h but for color components. \n"
      " For most images value equals 10");

  ctlbind(ctls, "templateWindowRadius", ctx(&this_class::templateWindowRadius),
      "Radius in pixels of the template patch that is used to compute weights.");

  ctlbind(ctls, "searchWindowRadius", ctx(&this_class::searchWindowRadius),
      "Size in pixels of the window that is used to compute weighted average for given pixel");

}

bool c_fast_nl_means_denoising_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, h);
    SERIALIZE_OPTION(settings, save, *this, hColor);
    SERIALIZE_OPTION(settings, save, *this, templateWindowRadius);
    SERIALIZE_OPTION(settings, save, *this, searchWindowRadius);
    return true;
  }
  return false;
}

bool c_fast_nl_means_denoising_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
#if HAVE_OpenCV_photo
  cv::fastNlMeansDenoisingColored(image.getMat(), image, h, hColor,
      2 * templateWindowRadius + 1,
      2 * searchWindowRadius + 1);
  return true;
#else
  CF_ERROR("OpenCV module photo is not available. Can not call cv::fastNlMeansDenoisingColored()");
  (void)(image);
  (void)(mask);
  return false;
#endif
}
