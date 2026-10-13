/*
 * c_apodization_routine.cc
 *
 *  Created on: Oct 12, 2026
 *      Author: amyznikov
 */

#include "c_apodization_routine.h"
#include <core/proc/fft.h>
#include <core/proc/multiply.h>
#include <core/proc/inpaint.h>

bool c_apodization_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, edgeSize);
    SERIALIZE_OPTION(settings, save, *this, lvls);
    return true;
  }
  return false;
}

void c_apodization_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "edgeSize [px]: ", CTL_CONTEXT(ctx, edgeSize), "");
  ctlbind(ctls, "lvls: ", CTL_CONTEXT(ctx, lvls), "");

}

extern cv::Mat1f fftCreateEllipticalApodizationWindow(const cv::Size & size, double edgeSize);

bool c_apodization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( apodizationWindow.size() != image.size() || edgeSize != oldEdgeSize ) {
    apodizationWindow = fftCreateEllipticalApodizationWindow(image.size(), oldEdgeSize = edgeSize);
  }
  multiplyImages(image, apodizationWindow, image);
  return true;
}
