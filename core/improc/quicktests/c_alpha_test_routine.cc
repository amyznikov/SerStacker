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
      { c_alpha_test_routine::DISPLAY_DRIZZLED_IMAGE, "DRIZZLED_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_DRIZZLE_ACCUMULATOR, "DRIZZLE_ACCUMULATOR", "" },
      { c_alpha_test_routine::DISPLAY_DRIZZLE_WEIGHTS, "DRIZZLE_WEIGHTS", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {
} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _drizzleScale);
    SERIALIZE_OPTION(settings, save, *this, _drizzlePixFrac);

    if ( auto group = SERIALIZE_GROUP(settings, save, "Translation")) {
      SERIALIZE_OPTION(group, save, translation, T);
    }

    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "scale", CTL_CONTEXT(ctx, _drizzleScale), "");
  ctlbind(ctls, "pixfrac", CTL_CONTEXT(ctx, _drizzlePixFrac), "");

  ctlbind_expandable_group(ctls, "Translation Options",
      [&, ctx = CTL_CONTEXT(ctx, translation )]() {
        ctlbind(ctls, "Translation (X;Y):", CTL_CONTEXT(ctx, T), "");
      });
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_CURRENT_IMAGE ) {
    return true;
  }

  cv::Mat src, acc, accw;
  c_translation_image_transform transfrom(translation.T);

  if ( image.depth() == CV_32F ) {
    src = image.getMat();
  }
  else {
    image.getMat().convertTo(src, CV_32F);
  }

  if ( !transfrom.drizzle(src, acc, accw, _drizzleScale, _drizzlePixFrac) ) {
    CF_ERROR("transfrom.drizzle() fails");
    return false;
  }

  switch (_display) {
    case DISPLAY_DRIZZLE_ACCUMULATOR:
      image.assign(acc);
      mask.release();
      break;
    case DISPLAY_DRIZZLE_WEIGHTS:
      image.assign(accw);
      mask.release();
      break;
    case DISPLAY_DRIZZLED_IMAGE:
      divideImages(acc, accw, image);
      mask.release();
      break;
    default:
      break;
  }

  return true;
}


