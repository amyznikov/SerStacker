/*
 * c_type_convert_routine.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_type_convert_routine_h__
#define __c_type_convert_routine_h__

#include "c_image_processor.h"
#include <core/proc/pixtype.h>

class c_type_convert_routine
    : public c_image_processor_routine
{
public:
  typedef c_type_convert_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("convertTo", "convertTo",
            "calls \n"
            "image.convertTo(\n"
            "  ddepth, \n"
            "  alpha, \n"
            "  beta)",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_type_convert_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(PIXEL_DEPTH ddepth, double alpha, double beta, bool enabled = true);

  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_ddepth(enum PIXEL_DEPTH v);
  enum PIXEL_DEPTH ddepth() const;

  void set_alpha(double  v);
  double alpha() const;

  void set_beta(double  v);
  double beta() const;

  void set_auto_scale(bool v);
  bool auto_scale() const;


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ddepth, "OpenCV pixel depth");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, alpha, "scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, beta, "offset");
  }

protected:
  double alpha_ = 1.0;
  double beta_ = 0.0;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_32F;
  bool auto_scale_ = true;
};



#endif /* __c_type_convert_routine_h__ */

