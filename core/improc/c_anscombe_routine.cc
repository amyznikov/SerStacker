/*
 * c_anscombe_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_anscombe_routine.h"

c_anscombe_routine::c_class_factory c_anscombe_routine::class_factory;

c_anscombe_routine::c_anscombe_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_anscombe_routine::ptr c_anscombe_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_anscombe_routine::ptr c_anscombe_routine::create(enum anscombe_method m, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_method(m);
  return obj;
}

bool c_anscombe_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  anscombe_.apply(image.getMatRef(), image.getMatRef());
  return true;
}

void c_anscombe_routine::set_method(enum anscombe_method v)
{
  anscombe_.set_method(v);
}

enum anscombe_method c_anscombe_routine::method() const
{
  return anscombe_.method();
}

bool c_anscombe_routine::load(c_config_setting settings)
{
  if ( !base::load(settings) ) {
    return false;
  }

  int v = anscombe_.method();
  if ( settings.get("method", &v) ) {
    anscombe_.set_method((anscombe_method) (v));
  }

  return true;
}

bool c_anscombe_routine::save(c_config_setting settings) const
{
  if ( !base::save(settings) ) {
    return false;
  }

  settings.set("method", (int)anscombe_.method());

  return true;
}

