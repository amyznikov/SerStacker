/*
 * c_upject_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_upject_routine_h__
#define __c_upject_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/downstrike.h>

class c_upject_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_upject_routine,
      "upject",
      "2x upsampling step by injecting EVEN or UNEVEN rows and columns");

  enum UpjectMode {
    UpjectUneven,
    UpjectEven,
  };

  enum FillMode {
    FillZero,
    FillAvg,
  };

  void set_mode(UpjectMode v)
  {
    mode_ = v;
  }

  UpjectMode mode() const
  {
    return mode_;
  }

  void set_fill_mode(FillMode v)
  {
    fill_mode_ = v;
  }

  FillMode fill_mode() const
  {
    return fill_mode_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override;

protected:
  UpjectMode mode_ = UpjectUneven;
  FillMode fill_mode_ = FillAvg;
  cv::Size dstSize_ = cv::Size (-1,-1);
};

#endif /* __c_upject_routine_h__ */
