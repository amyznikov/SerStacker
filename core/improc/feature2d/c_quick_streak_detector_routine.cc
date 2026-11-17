/*
 * c_quick_streak_detector_routine.cc
 *
 *  Created on: Nov 17, 2026
 *      Author: amyznikov
 */

#include "c_quick_streak_detector_routine.h"

template<>
const c_enum_member * members_of<c_quick_streak_detector_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_quick_streak_detector_routine::DISPLAY_SRC, "SRC", "" },
      { c_quick_streak_detector_routine::DISPLAY_TOPHAT, "TOPHAT", "" },
      { c_quick_streak_detector_routine::DISPLAY_TOPMASK, "TOPMASK", "" },
      { c_quick_streak_detector_routine::DISPLAY_TOPMASK}
  };
  return members;
}

bool c_quick_streak_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _update_roi);
    serialize_quick_streak_checker_options(settings, save, _opts);
    return true;
  }
  return false;
}

void c_quick_streak_detector_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, CTL_CONTEXT(ctx, _opts));
  ctlbind(ctls, "update_roi",  CTL_CONTEXT(ctx, _update_roi));
}

bool c_quick_streak_detector_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Vec4i T;

  const bool hasStreaks =
      _streak_checker.check_for_streaks(_opts,
          image, mask, &T);

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

  if ( !hasStreaks ) {
    CF_DEBUG("hasStreaks: %d median=%g sigma=%g threshold=%g", hasStreaks,
        _streak_checker.median(),
        _streak_checker.sigma(),
        _streak_checker.threshold());

  }
  else {

    CF_DEBUG("hasStreaks: %d median=%g sigma=%g threshold=%g Track: {%d %d %d %d}", hasStreaks,
        _streak_checker.median(),
        _streak_checker.sigma(),
        _streak_checker.threshold(),
        T(0), T(1), T(2), T(3));

    if ( _update_roi ) {
      const cv::Rect streakRoi(std::min(T(0), T(2)), std::min(T(1), T(3)),
          std::abs(T(2) - T(0)) + 1, std::abs(T(3) - T(1)) + 1);
      ctlbind_update_roi(streakRoi);
    }
  }

  return true;
}
