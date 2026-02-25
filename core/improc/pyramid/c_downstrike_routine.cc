/*
 * c_downstrike_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_downstrike_routine.h"

template<>
const c_enum_member * members_of<c_downstrike_routine::DownstrikeMode>()
{
  static const c_enum_member members[] = {
      {c_downstrike_routine::DownstrikeUneven, "Uneven"},
      {c_downstrike_routine::DownstrikeEven, "Even"},
      {c_downstrike_routine::DownstrikeUneven},
  };

  return members;
}

void c_downstrike_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "mode",  ctx(&this_class::_mode), "Which rows and columns to reject");
}

bool c_downstrike_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _mode);
    return true;
  }
  return false;
}

bool c_downstrike_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat img;

  switch (_mode) {
    case DownstrikeUneven:
      downstrike_uneven(image.getMat(), img);
      image.move(img);
      if( mask.needed() && !mask.empty() ) {
        downstrike_uneven(mask.getMat(), img);
        mask.move(img);
      }
      break;
    case DownstrikeEven:
      downstrike_even(image.getMat(), img);
      image.move(img);
      if( mask.needed() && !mask.empty() ) {
        downstrike_even(mask.getMat(), img);
        mask.move(img);
      }
      break;
    default:
      CF_ERROR("Invalid downstrike mode specified");
      return false;
  }
  return true;
}
