/*
 * c_downstrike_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_downstrike_routine_h__
#define __c_downstrike_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/downstrike.h>

class c_downstrike_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_downstrike_routine,
      "downstrike",
      "2x downsampling step by rejecting each even (uneven) row and column, keep only uneven (even)");

  enum DownstrikeMode {
    DownstrikeUneven,
    DownstrikeEven,
  };

  void set_mode(DownstrikeMode v)
  {
    mode_ = v;
  }

  DownstrikeMode mode() const
  {
    return mode_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, mode, "Which rows and columns to reject");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override
  {
    cv::Mat img;

    switch (mode_) {
      case DownstrikeUneven:
        downstrike_uneven(image.getMat(), img);
        image.move(img);
        if( mask.needed() && !mask.empty() ) {
          downstrike_uneven(mask.getMat(), img);
          mask.move(img);
        }
        break;
      case DownstrikeEven:
        downstrike_even(image.getMat(), img);
        image.move(img);
        if( mask.needed() && !mask.empty() ) {
          downstrike_even(mask.getMat(), img);
          mask.move(img);
        }
        break;
      default:
        CF_ERROR("Invalid downstrike mode specified");
        return false;
    }
    return true;
  }

protected:
  DownstrikeMode mode_ = DownstrikeUneven;
};

#endif /* __c_downstrike_routine_h__ */
