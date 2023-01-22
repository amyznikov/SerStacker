/*
 * c_range_normalize_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_range_normalize_routine_h__
#define __c_range_normalize_routine_h__

#include "c_image_processor.h"
#include <core/proc/normalize.h>

class c_range_normalize_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_range_normalize_routine, "normalize",
      "cv::normalize()");

  void set_auto_input_range(bool v)
  {
    auto_input_range_ = v;
  }

  bool auto_input_range() const
  {
    return auto_input_range_;
  }

  void set_input_min(double  v)
  {
    input_min_ = v;
  }

  double input_min() const
  {
    return input_min_;
  }

  void set_input_max(double  v)
  {
    input_max_ = v;
  }

  double input_max() const
  {
    return input_max_;
  }

  void set_output_min(double v)
  {
    output_min_ = v;
  }

  double output_min() const
  {
    return output_min_;
  }

  void set_output_max(double v)
  {
    output_max_ = v;
  }

  double output_max() const
  {
    return output_max_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, auto_input_range, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, input_min, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, input_max, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_min, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output_max, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, auto_input_range);
      SERIALIZE_PROPERTY(settings, save, *this, input_min);
      SERIALIZE_PROPERTY(settings, save, *this, input_max);
      SERIALIZE_PROPERTY(settings, save, *this, output_min);
      SERIALIZE_PROPERTY(settings, save, *this, output_max);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    bool fOk;

    // CF_DEBUG("auto_input_range_=%d", auto_input_range_);

    if ( auto_input_range_ ) {

      // CF_DEBUG("normalize_minmax(output_min_=%g, output_max_=%g)", output_min_, output_max_);
      fOk = normalize_minmax(image.getMatRef(),
          image.getMatRef(),
          output_min_,
          output_max_,
          mask);
    }
    else {

      // CF_DEBUG("normalize_image(input_min_=%g input_max_=%g output_min_=%g, output_max_=%g)", input_min_, input_max_, output_min_, output_max_);
      fOk = normalize_image(image.getMatRef(),
          input_min_,
          input_max_,
          output_min_,
          output_max_,
          mask.getMatRef());

    }

    return fOk;
  }

protected:
  double  input_min_ = 0.0;
  double  input_max_ = 1.0;
  double  output_min_ = 0.0;
  double  output_max_ = 1.0;
  bool    auto_input_range_ = true;
};

#endif /* __c_range_normalize_routine_h__ */
