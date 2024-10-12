/*
 * c_type_convertor_routine.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#include "c_type_convertor_routine.h"

void c_type_convertor_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, ddepth, "OpenCV pixel depth");
  BIND_PCTRL(ctls, auto_scale, "auto_scale");
  BIND_PCTRL(ctls, alpha, "scale");
  BIND_PCTRL(ctls, beta, "offset");
}

bool c_type_convertor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, ddepth);
    SERIALIZE_PROPERTY(settings, save, *this, alpha);
    SERIALIZE_PROPERTY(settings, save, *this, beta);
    SERIALIZE_PROPERTY(settings, save, *this, auto_scale);
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
