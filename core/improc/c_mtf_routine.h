/*
 * c_mtf_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mtf_routine_h__
#define __c_mtf_routine_h__

#include <core/mtf/c_pixinsight_mtf.h>
#include "c_image_processor.h"

class c_mtf_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mtf_routine,
      "mtf", "mtf");

  c_pixinsight_mtf & mtf()
  {
    return mtf_;
  }

  const c_pixinsight_mtf & mtf() const
  {
    return mtf_;
  }

  void set_shadows(double v)
  {
    mtf_.set_shadows(v);
  }

  double shadows() const
  {
    return mtf_.shadows();
  }

  void set_highlights(double v)
  {
    mtf_.set_highlights(v);
  }

  double highlights() const
  {
    return mtf_.highlights();
  }

  void set_midtones(double v)
  {
    mtf_.set_midtones(v);
  }

  double midtones() const
  {
    return mtf_.midtones();
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, shadows, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, highlights, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, midtones, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, shadows);
      SERIALIZE_PROPERTY(settings, save, *this, highlights);
      SERIALIZE_PROPERTY(settings, save, *this, midtones);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return mtf_.apply(image, image);
  }

protected:
  c_pixinsight_mtf mtf_;
};

#endif /* __c_mtf_routine_h__ */
