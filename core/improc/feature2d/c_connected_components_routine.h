/*
 * c_connected_components_routine.h
 *
 *  Created on: May 17, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_connected_components_routine_h__
#define __c_connected_components_routine_h__

#include <core/improc/c_image_processor.h>

class c_connected_components_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_connected_components_routine,
      "connected_components", "connected components labeling");

  enum Connectivity {
    connectivity4 = 4,
    connectivity8 = 8,
  };

  void set_connectivity(Connectivity v)
  {
    _connectivity = v;
  }

  Connectivity connectivity() const
  {
    return _connectivity;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  Connectivity _connectivity = connectivity4;
};

#endif /* __c_connected_components_routine_h__ */
