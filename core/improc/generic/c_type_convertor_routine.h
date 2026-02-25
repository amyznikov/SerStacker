/*
 * c_type_convertor_routine.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_type_convertor_routine_h__
#define __c_type_convertor_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_type_convertor_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_type_convertor_routine,
      "convertTo",
      "Calls image.convertTo(ddepth, alpha, beta)")  ;

  bool serialize(c_config_setting settings, bool save)  final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _alpha = 1.0;
  double _beta = 0.0;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_32F;
  bool _auto_scale = true;
};



#endif /* __c_type_convertor_routine_h__ */

