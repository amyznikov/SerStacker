/*
 * c_anscombe_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_anscombe_routine_h__
#define __c_anscombe_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_anscombe_transform.h>

class c_anscombe_routine
: public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_anscombe_routine,
      "anscombe", "Apply anscombe transform to image");

  void set_method(enum anscombe_method v)
  {
    anscombe_.set_method(v);
  }

  enum anscombe_method method() const
  {
    return anscombe_.method();
  }

  void set_invert(bool v)
  {
    invert_ = v;
  }

  bool invert() const
  {
    return invert_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, method, "");
    BIND_PCTRL(ctls, invert, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, anscombe_, method);
      SERIALIZE_PROPERTY(settings, save, *this, invert);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if ( invert_ ) {
      anscombe_.inverse(image.getMatRef(), image.getMatRef());
    }
    else {
      anscombe_.apply(image.getMatRef(), image.getMatRef());
    }

    return true;
  }

protected:
  c_anscombe_transform anscombe_;
  bool invert_ = false;
};



#endif /* __c_anscombe_routine_h__ */
