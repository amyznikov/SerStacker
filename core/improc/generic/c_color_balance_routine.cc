/*
 * c_color_balance_routine.cc
 *
 *  Created on: Sep 28, 2023
 *      Author: amyznikov
 */

#include "c_color_balance_routine.h"

void c_color_balance_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "gscale", ctx(&this_class::_gscale), "");
  ctlbind(ctls, "alpha", ctx(&this_class::_alpha), "");
}

bool c_color_balance_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _alpha);
    SERIALIZE_OPTION(settings, save, *this, _gscale);
    return true;
  }
  return false;
}

bool c_color_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() && image.channels() == 3 ) {

    const double v =
        _alpha > 0 ? _alpha * _alpha :
            -_alpha * _alpha;

    const double b = (1 + v );
    const double g = _gscale;
    const double r = (1 - v);

    cv::transform(image.getMat(), image,
        cv::Matx33f(
            b, 0, 0,
            0, g, 0,
            0, 0, r
            ));
  }

  return true;
}
