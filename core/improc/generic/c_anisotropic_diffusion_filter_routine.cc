/*
 * c_anisotropic_diffusion_filter_routine.cc
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#include "c_anisotropic_diffusion_filter_routine.h"
#include <opencv2/ximgproc.hpp>

void c_anisotropic_diffusion_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "filterType", ctx(&this_class::alpha),
      "The amount of time to step forward by on each iteration (normally, it's between 0 and 1).");

  ctlbind(ctls, "K", ctx(&this_class::K),
      "Sensitivity to the edges");

  ctlbind(ctls, "niters", ctx(&this_class::niters),
      "The number of iterations");

}

bool c_anisotropic_diffusion_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, alpha);
    SERIALIZE_OPTION(settings, save, *this, K);
    SERIALIZE_OPTION(settings, save, *this, niters);
    return true;
  }
  return false;
}

bool c_anisotropic_diffusion_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::ximgproc::anisotropicDiffusion(image.getMat(), image, alpha, K, niters);
  return true;
}
