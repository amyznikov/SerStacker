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


  void set_ddepth(enum PIXEL_DEPTH v)
  {
    _ddepth = v;
  }

  enum PIXEL_DEPTH ddepth() const
  {
    return _ddepth;
  }

  void set_alpha(double  v)
  {
    _alpha = v;
  }

  double alpha() const
  {
    return _alpha;
  }

  void set_beta(double  v)
  {
    _beta = v;
  }

  double beta() const
  {
    return _beta;
  }

  void set_auto_scale(bool v)
  {
    _auto_scale = v;
  }

  bool auto_scale() const
  {
    return _auto_scale;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls)  final;
  bool serialize(c_config_setting settings, bool save)  final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double _alpha = 1.0;
  double _beta = 0.0;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_32F;
  bool _auto_scale = true;
};



#endif /* __c_type_convertor_routine_h__ */

