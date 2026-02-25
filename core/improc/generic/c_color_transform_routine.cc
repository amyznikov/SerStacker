/*
 * c_color_transform_routine.cc
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */

#include "c_color_transform_routine.h"

void c_color_transform_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "rb", ctx(&this_class::_rb), "");
  ctlbind(ctls, "rg", ctx(&this_class::_rg), "");
  ctlbind(ctls, "rr", ctx(&this_class::_rr), "");
}

bool c_color_transform_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _rb);
    SERIALIZE_OPTION(settings, save, *this, _rg);
    SERIALIZE_OPTION(settings, save, *this, _rr);
    return true;
  }
  return false;
}

bool c_color_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const float cx = std::cos(_rb * CV_PI / 180);
  const float sx = std::sin(_rb * CV_PI / 180);
  const cv::Matx33f Rx(
      1, 0, 0,
      0, cx, -sx,
      0, sx, cx);

  const float cy = std::cos(_rg * CV_PI / 180);
  const float sy = std::sin(_rg * CV_PI / 180);
  const cv::Matx33f Ry(
      cy, 0, sy,
      0, 1, 0,
      -sy, 0, cy);

  const float cz = std::cos(_rr * CV_PI / 180);
  const float sz = std::sin(_rr * CV_PI / 180);
  const cv::Matx33f Rz(
      cz, -sz, 0,
      sz, cz, 0,
      0, 0, 1);

  const cv::Matx33f m =
      Rx * Ry * Rz;

  cv::transform(image, image, m);

  return true;
}

