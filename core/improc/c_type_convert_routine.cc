/*
 * c_type_convert_routine.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#include "c_type_convert_routine.h"


c_type_convert_routine::c_class_factory c_type_convert_routine::class_factory;

c_type_convert_routine::c_type_convert_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_type_convert_routine::ptr c_type_convert_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_type_convert_routine::ptr c_type_convert_routine::create(PIXEL_DEPTH ddepth, double alpha, double beta, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_ddepth(ddepth);
  obj->set_alpha(alpha);
  obj->set_beta(beta);
  return obj;
}

void c_type_convert_routine::set_ddepth(PIXEL_DEPTH v)
{
  ddepth_ = v;
}

PIXEL_DEPTH c_type_convert_routine::ddepth() const
{
  return ddepth_;
}

void c_type_convert_routine::set_alpha(double v)
{
  alpha_ = v;
}

double c_type_convert_routine::alpha() const
{
  return alpha_;
}

void c_type_convert_routine::set_beta(double v)
{
  beta_ = v;
}

double c_type_convert_routine::beta() const
{
  return beta_;
}

void c_type_convert_routine::set_auto_scale(bool v)
{
  auto_scale_ = v;
}

bool c_type_convert_routine::auto_scale() const
{
  return auto_scale_;
}

bool c_type_convert_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int ddepth = (ddepth_ == PIXEL_DEPTH_NO_CHANGE || image.fixedType()) ?
      image.depth() : ddepth_;

  double alpha, beta;

  if ( !auto_scale_ ) {
    alpha = alpha_;
    beta = beta_;
  }
  else {

    double src_max = 1;
    double dst_max = 1;

    switch (image.depth()) {
    case CV_8U:
      src_max = UINT8_MAX;
      break;
    case CV_8S:
      src_max = INT8_MAX;
      break;
    case CV_16U:
      src_max = UINT16_MAX;
      break;
    case CV_16S:
      src_max = INT16_MAX;
      break;
    case CV_32S:
      src_max = INT32_MAX;
      break;
    case CV_32F:
      src_max = 1;
      break;
    case CV_64F:
      src_max = 1;
      break;
    }

    switch (ddepth) {
    case CV_8U:
      dst_max = UINT8_MAX;
      break;
    case CV_8S:
      dst_max = INT8_MAX;
      break;
    case CV_16U:
      dst_max = UINT16_MAX;
      break;
    case CV_16S:
      dst_max = INT16_MAX;
      break;
    case CV_32S:
      dst_max = INT32_MAX;
      break;
    case CV_32F:
      dst_max = 1;
      break;
    case CV_64F:
      dst_max = 1;
      break;
    }

    alpha = dst_max / src_max;
    beta = 0;
  }

  if( ddepth != image.depth() || alpha != 1 || beta != 0 ) {

    cv::Mat src = image.getMat();

    src.convertTo(image, ddepth, alpha, beta);
  }

  return true;
}

bool c_type_convert_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, *this, ddepth);
  LOAD_PROPERTY(settings, *this, alpha);
  LOAD_PROPERTY(settings, *this, beta);
  LOAD_PROPERTY(settings, *this, auto_scale);

  return true;
}

bool c_type_convert_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }


  SAVE_PROPERTY(settings, *this, ddepth);
  SAVE_PROPERTY(settings, *this, alpha);
  SAVE_PROPERTY(settings, *this, beta);
  SAVE_PROPERTY(settings, *this, auto_scale);

  return true;
}
