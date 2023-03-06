/*
 * c_homography_test_routine.h
 *
 *  Created on: Mar 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_homography_test_routine_h__
#define __c_homography_test_routine_h__

#include "c_image_processor.h"

class c_homography_test_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_homography_test_routine,
      "homography_test", "homography_test");

  void set_rx(double v)
  {
    rx_ = v;
  }

  double rx() const
  {
    return rx_;
  }

  void set_ry(double v)
  {
    ry_ = v;
  }

  double ry() const
  {
    return ry_;
  }

  void set_rz(double v)
  {
    rz_ = v;
  }

  double rz() const
  {
    return rz_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rx, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ry, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rz, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, rx);
      SERIALIZE_PROPERTY(settings, save, *this, ry);
      SERIALIZE_PROPERTY(settings, save, *this, rz);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double rx_ = 0;
  double ry_ = 0;
  double rz_ = 0;
};

#endif /* __c_homography_test_routine_h__ */
