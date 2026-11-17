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
#include <core/proc/median.h>


template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_TOPHAT, "TOPHAT", "" },
      { c_alpha_test_routine::DISPLAY_TOPMASK, "TOPMASK", "" },
      { c_alpha_test_routine::DISPLAY_TOPMASK}
  };
  return members;
}

/////////////////////////////////////
namespace {
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    serialize_quick_streak_checker_options(settings, save, _opts);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, CTL_CONTEXT(ctx, _opts));
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const bool hasStreaks =
      _streak_checker.check_for_streaks(_opts,
          image, mask);

  switch (_display) {
    case DISPLAY_TOPHAT:
      image.assign(_streak_checker.tophat());
      break;
    case DISPLAY_TOPMASK:
      image.assign(_streak_checker.topmask());
      break;
    case DISPLAY_SRC:
      default:
      break;
  }

  CF_DEBUG("hasStreaks: %d median=%g sigma=%g threshold=%g", hasStreaks,
      _streak_checker.median(),
      _streak_checker.sigma(),
      _streak_checker.threshold());

  return true;
}
