/*
 * c_pnormalize_routine.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 *
 *  Subtract local mean and divide by local stdev
 */

#pragma once
#ifndef __c_pnormalize_routine_h__
#define __c_pnormalize_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/pyrscale.h>

class c_pnormalize_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pnormalize_routine, "pnormalize",
      "Subtract local mean and divide by stdev");

  void set_scale(int v)
  {
    scale_ = v;
  }

  int scale() const
  {
    return scale_;
  }

  void set_ddepth(PIXEL_DEPTH v)
  {
    ddepth_ = v;
  }

  PIXEL_DEPTH ddepth() const
  {
    return ddepth_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, scale, "Gaussian pyramid scale (max level)");
    BIND_PCTRL(ctls, ddepth, "Destination image depth");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, scale);
      SERIALIZE_PROPERTY(settings, save, *this, ddepth);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if( scale_ > 0 ) {
      pnormalize(image.getMat(), image.getMatRef(), scale_, ddepth_);
    }

    return true;
  }

  static void pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale, PIXEL_DEPTH ddepth);

protected:
  // cv::Mat m;
  int scale_ = 3;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_32F;

};

#endif /* __c_pnormalize_routine_h__ */
