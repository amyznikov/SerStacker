/*
 * c_autoclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_autoclip_routine_h__
#define __c_autoclip_routine_h__

#include "c_image_processor.h"
#include <core/proc/autoclip.h>

class c_autoclip_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_autoclip_routine,
      "autoclip_routine", "Auto clip image histogram");

  void set_lclip(double v)
  {
    plo_ = v;
  }

  double lclip() const
  {
    return plo_;
  }


  void set_hclip(double v)
  {
    phi_ = v;
  }

  double hclip() const
  {
    return phi_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lclip, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, hclip, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, lclip);
      SERIALIZE_PROPERTY(settings, save, *this, hclip);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    double omin = 0, omax = 1;

    switch ( image.depth() ) {
    case CV_8U :
      omin = 0, omax = UINT8_MAX;
      break;
    case CV_8S :
      omin = INT8_MIN, omax = INT8_MAX;
      break;
    case CV_16U :
      omin = 0, omax = UINT16_MAX;
      break;
    case CV_16S :
      omin = INT16_MIN, omax = INT16_MAX;
      break;
    case CV_32S :
      omin = INT32_MIN, omax = INT32_MAX;
      break;
      break;
    }

    return autoclip(image.getMatRef(), mask, plo_, phi_, omin, omax);
  }


protected:
  double plo_ = 0.5;
  double phi_ = 99.5;
};


#endif /* __c_autoclip_routine_h__ */
