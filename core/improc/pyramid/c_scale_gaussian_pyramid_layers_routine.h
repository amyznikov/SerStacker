/*
 * c_scale_gaussian_pyramid_layers_routine.h
 *
 *  Created on: Jul 13, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gaussian_pyramid_routine_h__
#define __c_gaussian_pyramid_routine_h__

#include <core/improc/c_image_processor.h>

class c_scale_gaussian_pyramid_layers_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_scale_gaussian_pyramid_layers_routine,
      "scale_pyramid_layers", "Decompose image into gaussian pyramid, "
          "scale layers and recompose image back");

  void set_scales(const std::vector<double> & scales)
  {
    scales_ = scales;
  }

  const std::vector<double> & scales() const
  {
    return scales_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, scales, "");
    BIND_PCTRL(ctls, borderType, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, scales);
      SERIALIZE_PROPERTY(settings, save, *this, borderType);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::vector<double> scales_;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;

};

#endif /* __c_gaussian_pyramid_routine_h__ */
