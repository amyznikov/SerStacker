/*
 * c_laplacian_map_routine.h
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_map_routine_h__
#define __c_laplacian_map_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/sharpness_measure/c_laplacian_sharpness_measure.h>

class c_laplacian_map_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_laplacian_map_routine,
      "laplacian_map", "c_laplacian_sharpness_measure map");

  void set_dscale(int v)
  {
    m_.set_dscale(v);
  }

  int dscale() const
  {
    return m_.dscale();
  }

  void set_se_size(const cv::Size & v)
  {
    m_.set_se_size(v);
  }

  const cv::Size & se_size() const
  {
    return m_.se_size();
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, dscale, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, se_size, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, dscale);
      SERIALIZE_PROPERTY(settings, save, *this, se_size);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    m_.create_map(image.getMat(), image);
    return true;
  }

protected:
  c_laplacian_sharpness_measure m_;
};

#endif /* __c_laplacian_map_routine_h__ */
