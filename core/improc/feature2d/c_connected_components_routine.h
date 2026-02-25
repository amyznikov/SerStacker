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
      "connected_components", "Apply cv::connectedComponents() for connected components labeling");

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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  Connectivity _connectivity = connectivity4;
};

#endif /* __c_connected_components_routine_h__ */
