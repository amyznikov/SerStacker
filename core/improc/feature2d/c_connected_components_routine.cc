/*
 * c_connected_components_routine.cc
 *
 *  Created on: May 17, 2025
 *      Author: amyznikov
 */

#include "c_connected_components_routine.h"

template<>
const c_enum_member * members_of<c_connected_components_routine::Connectivity>()
{
  static const c_enum_member members[] = {
    {c_connected_components_routine::connectivity4, "4", "4-way pixel connectivity"},
    {c_connected_components_routine::connectivity8, "8", "8-way pixel connectivity"},
    {c_connected_components_routine::connectivity4}
  };

  return members;
}

void c_connected_components_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "connectivity", ctx(&this_class::_connectivity), "");
  ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "");
}

bool c_connected_components_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, connectivity);
    return true;
  }
  return false;
}

bool c_connected_components_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( mask.empty() || _ignore_mask ) {

    cv::connectedComponents(image.getMat(),
        image,
        _connectivity);
  }
  else {
    cv::connectedComponents(image.getMat() & mask.getMat(),
        image,
        _connectivity);
  }

  return true;
}


