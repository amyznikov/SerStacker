/*
 * c_adaptive_gaussian_blur_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_adaptive_gaussian_blur_routine.h"

void c_adaptive_gaussian_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "sigma_hpass", CTL_CONTEXT(ctx, sigma_hpass), "");
  ctlbind(ctls, "sigma_lpass", CTL_CONTEXT(ctx, sigma_lpass), "");
  ctlbind(ctls, "kscale", CTL_CONTEXT(ctx, kscale), "");
  ctlbind(ctls, "kradius", CTL_CONTEXT(ctx, kradius), "");
  ctlbind(ctls, "display", ctx(&this_class::displayType), "Select output image");
}

bool c_adaptive_gaussian_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, displayType);
    SERIALIZE_OPTION(settings, save, *this, sigma_hpass);
    SERIALIZE_OPTION(settings, save, *this, sigma_lpass);
    SERIALIZE_OPTION(settings, save, *this, kscale);
    SERIALIZE_OPTION(settings, save, *this, kradius);
    return true;
  }
  return false;
}

bool c_adaptive_gaussian_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  adaptive_gaussian_blur(image, image,
      sigma_hpass, sigma_lpass, kscale, kradius,
      displayType);

  return true;
}
