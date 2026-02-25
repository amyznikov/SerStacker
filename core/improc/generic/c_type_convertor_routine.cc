/*
 * c_type_convertor_routine.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#include "c_type_convertor_routine.h"

void c_type_convertor_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "Output OpenCV pixel depth");
   ctlbind(ctls, "alpha", ctx(&this_class::_alpha), "output = alpha * input + beta");
   ctlbind(ctls, "beta", ctx(&this_class::_beta), "");
   ctlbind(ctls, "auto_scale", ctx(&this_class::_auto_scale), "auto alpha/beta");
}

bool c_type_convertor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _ddepth);
    SERIALIZE_OPTION(settings, save, *this, _alpha);
    SERIALIZE_OPTION(settings, save, *this, _beta);
    SERIALIZE_OPTION(settings, save, *this, _auto_scale);
    return true;
  }
  return false;
}

bool c_type_convertor_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int ddepth =
      (_ddepth == PIXEL_DEPTH_NO_CHANGE || image.fixedType()) ?
          image.depth() :
          _ddepth;

  double alpha, beta;

  if( !_auto_scale ) {
    alpha = _alpha;
    beta = _beta;
  }
  else {
    get_scale_offset(image.depth(), _ddepth,
        &alpha, &beta);
  }

  if( ddepth != image.depth() || alpha != 1 || beta != 0 ) {

    image.getMat().convertTo(image, ddepth,
        alpha, beta);
  }

  return true;
}
