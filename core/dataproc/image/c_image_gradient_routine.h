/*
 * c_image_gradient_routine.h
 *
 *  Created on: Jan 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_gradient_routine_h__
#define __c_image_gradient_routine_h__

#include <core/dataproc/c_data_frame_processor.h>

class c_image_gradient_routine :
    public c_data_frame_processor_routine
{
public:
  DECLARE_DATA_PROCESSOR_CLASS_FACTORY(c_image_gradient_routine,
      "gradient",
      "compute image gradient");

  const std::string & input_image_name() const;
  void set_input_image_name(const std::string & v);

  const std::string & output_image_name() const;
  void set_output_image_name(const std::string & v);

  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_data_frame::sptr & dataframe) final;

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
    ctlbind(ctls, "input_image", ctx(&this_class::_input_image_name), "");
    ctlbind(ctls, "output_image", ctx(&this_class::_output_image_name), "");
  }

protected:
  std::string _input_image_name;
  std::string _output_image_name;
};

#endif /* __c_image_gradient_routine_h__ */
