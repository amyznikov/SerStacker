/*
 * c_color_transform_routine.h
 *
 *  Created on: Mar 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_transform_routine_h__
#define __c_color_transform_routine_h__

#include "c_image_processor.h"

class c_color_transform_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_transform_routine,
      "color_transform", "Apply cv::transfrom() to image");

  void set_rb(float v)
  {
    rb_ = v;
  }

  float rb() const
  {
    return rb_;
  }

  void set_rg(float v)
  {
    rg_ = v;
  }

  float rg() const
  {
    return rg_;
  }

  void set_rr(float v)
  {
    rr_ = v;
  }

  float rr() const
  {
    return rr_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rb, "Rotation about B axis, deg");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rg, "Rotation about G axis, deg");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rr, "Rotation about R axis, deg");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, rb);
      SERIALIZE_PROPERTY(settings, save, *this, rg);
      SERIALIZE_PROPERTY(settings, save, *this, rr);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    const float cx = std::cos(rb_ * CV_PI / 180);
    const float sx = std::sin(rb_ * CV_PI / 180);
    const cv::Matx33f Rx(
        1, 0, 0,
        0, cx, -sx,
        0, sx, cx);

    const float cy = std::cos(rg_ * CV_PI / 180);
    const float sy = std::sin(rg_ * CV_PI / 180);
    const cv::Matx33f Ry(
        cy, 0, sy,
        0, 1, 0,
        -sy, 0, cy);

    const float cz = std::cos(rr_ * CV_PI / 180);
    const float sz = std::sin(rr_ * CV_PI / 180);
    const cv::Matx33f Rz(
        cz, -sz, 0,
        sz, cz, 0,
        0, 0, 1);

    const cv::Matx33f m =
        Rx * Ry * Rz;

    cv::transform(image, image, m);

    return true;
  }

protected:
  float rb_ = 0;
  float rg_ = 0;
  float rr_ = 0;
};

#endif /* __c_color_transform_routine_h__ */
