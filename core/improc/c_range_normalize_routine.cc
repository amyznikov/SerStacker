/*
 * c_range_normalize_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_range_normalize_routine.h"

c_range_normalize_routine::c_class_factory c_range_normalize_routine::class_factory;

c_range_normalize_routine::c_range_normalize_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_range_normalize_routine::ptr c_range_normalize_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_range_normalize_routine::ptr c_range_normalize_routine::create(double outmin, double outmax, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_output_min(outmin);
  obj->set_output_max(outmax);
  return obj;
}

void c_range_normalize_routine::set_input_min(double  v)
{
  input_min_ = v;
}

double c_range_normalize_routine::input_min() const
{
  return input_min_;
}

void c_range_normalize_routine::set_input_max(double v)
{
  input_max_ = v;
}

double c_range_normalize_routine::input_max() const
{
  return input_max_;
}

void c_range_normalize_routine::set_output_min(double v)
{
  output_min_ = v;
}

double c_range_normalize_routine::output_min() const
{
  return output_min_;
}

void c_range_normalize_routine::set_output_max(double v)
{
  output_max_ = v;
}

double c_range_normalize_routine::output_max() const
{
  return output_max_;
}

void c_range_normalize_routine::set_auto_input_range(bool v)
{
  auto_input_range_ = v;
}

bool c_range_normalize_routine::auto_input_range() const
{
  return auto_input_range_;
}

bool c_range_normalize_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  bool fOk;

  CF_DEBUG("auto_input_range_=%d", auto_input_range_);

  if ( auto_input_range_ ) {

    CF_DEBUG("normalize_minmax(output_min_=%g, output_max_=%g)", output_min_, output_max_);
    fOk = normalize_minmax(image.getMatRef(),
        image.getMatRef(),
        output_min_,
        output_max_,
        mask,
        true,
        cv::Scalar::all(0));
  }
  else {

    CF_DEBUG("normalize_image(input_min_=%g input_max_=%g output_min_=%g, output_max_=%g)", input_min_, input_max_, output_min_, output_max_);
    fOk = normalize_image(image.getMatRef(),
        input_min_,
        input_max_,
        output_min_,
        output_max_,
        mask.getMatRef(),
        true,
        cv::Scalar::all(0));

  }

  return fOk;
}

bool c_range_normalize_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("auto_input_range", &auto_input_range_);
  settings.get("input_min", &input_min_);
  settings.get("input_max", &input_max_);
  settings.get("output_min", &output_min_);
  settings.get("output_max", &output_max_);

  return true;
}

bool c_range_normalize_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("auto_input_range", auto_input_range_);
  settings.set("input_min", input_min_);
  settings.set("input_max", input_max_);
  settings.set("output_min", output_min_);
  settings.set("output_max", output_max_);


  return true;
}
