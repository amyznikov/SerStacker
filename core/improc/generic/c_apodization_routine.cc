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
    SERIALIZE_OPTION(settings, save, *this, zeroMean);
    return true;
  }
  return false;
}

void c_apodization_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "edgeSize [px]: ", CTL_CONTEXT(ctx, edgeSize), "");
  ctlbind(ctls, "zeroMean: ", CTL_CONTEXT(ctx, zeroMean), "");

}

bool c_apodization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( apodizationWindow.size() != image.size() || edgeSize != oldEdgeSize ) {
    fftCreateSuperEllipticalApodizationWindow(apodizationWindow, image.size(), oldEdgeSize = edgeSize);
  }

  if ( zeroMean ) {
    cv::Mat msk;
    if ( !mask.empty() ) {
      if ( mask.depth() == CV_8U ) {
        msk = mask.getMat();
      }
      else {
        cv::compare(mask, 0, msk, cv::CMP_GT);
      }
    }
    cv::subtract(image, cv::mean(image, msk), image);
  }

  multiplyImages(image, apodizationWindow, image);
  return true;
}
