/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>
#include <core/proc/divide.h>
#include <core/proc/multiply.h>



template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_FF_IMAGE, "FF_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_EQUALIZED_IMAGE, "EQUALIZED_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {

static void makeFFImage(cv::InputArray _src, cv::OutputArray _dst, int maxLvl = 2, double eps = 1e-4)
{
  const cv::Size srcSize = _src.size();
  cv::Mat src;

  if ( _src.channels() == 1 ) {
    src = _src.getMat();
  }
  else {
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
  }

  cv::Mat blured;

  for ( int i = 0; i < maxLvl; ++i ) {
    cv::medianBlur(i == 0 ? src : blured, blured, 5);
    cv::pyrDown(blured, blured);
  }

  // Don't use cv::resize() as it will always create crucial interpolation artifacts !
  for ( int i = 0; i < maxLvl; ++i ) {
    cv::pyrUp(blured, blured);
  }

  cv::add(blured, eps, _dst);
}

} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, maxLvl);
    SERIALIZE_OPTION(settings, save, *this, eps);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "maxLvl", CTL_CONTEXT(ctx, maxLvl), "Max pyramid level");
  ctlbind(ctls, "eps", CTL_CONTEXT(ctx, eps), "eps");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_CURRENT_IMAGE ) {
    return true;
  }

  cv::Mat ff;
  makeFFImage(image, ff, maxLvl, eps);
  if ( _display == DISPLAY_FF_IMAGE ) {
    image.move(ff);
    return true;
  }

  divideImages(image, ff, image);
  if ( _display == DISPLAY_EQUALIZED_IMAGE ) {
    return true;
  }

  multiplyImages(image, ff, image);

  return true;
}


