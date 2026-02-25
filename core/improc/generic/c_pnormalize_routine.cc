/*
 * c_pnormalize_routine.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_pnormalize_routine.h"

void c_pnormalize_routine::pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale, PIXEL_DEPTH ddepth)
{
  cv::Mat m;
  cv::Scalar mean, stdev;
  double f = 0;

  if( m.channels() == 1 ) {
    pyramid_downscale(src, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
  }
  else {
    cv::cvtColor(src, m, cv::COLOR_BGR2GRAY);
    pyramid_downscale(m, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
    cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
  }

  cv::subtract(src, m, dst, cv::noArray(), ddepth);

//
//  cv::meanStdDev(m, mean, stdev);
//  for( int i = 0, cn = src.channels(); i < cn; ++i ) {
//    f += stdev[i];
//  }
//
//  m.convertTo(dst, CV_8U, 24. * src.channels() / f, 128);
}

void c_pnormalize_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "scale", ctx(&this_class::_scale), "Gaussian pyramid scale (max level)");
   ctlbind(ctls, "ddepth", ctx(&this_class::_ddepth), "Destination image depth");
}

bool c_pnormalize_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _scale);
    SERIALIZE_OPTION(settings, save, *this, _ddepth);
    return true;
  }
  return false;
}

bool c_pnormalize_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _scale > 0 ) {
    pnormalize(image.getMat(), image.getMatRef(), _scale, _ddepth);
  }

  return true;
}

