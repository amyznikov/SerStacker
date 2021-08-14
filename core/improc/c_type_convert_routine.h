/*
 * c_type_convert_routine.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_type_convert_routine_h__
#define __c_type_convert_routine_h__

#include "c_image_processor.h"

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


  enum DDEPTH
  {
    DDEPTH_8U = CV_8U,
    DDEPTH_8S = CV_8S,
    DDEPTH_16U = CV_16U,
    DDEPTH_16S = CV_16S,
    DDEPTH_32S = CV_32S,
    DDEPTH_32F = CV_32F,
    DDEPTH_64F = CV_64F,
    DDEPTH_SAME = -1,
  };

  static const struct ddepth_desc {
    const char * name;
    enum DDEPTH value;
  } ddepths[];




  c_type_convert_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(DDEPTH ddepth, double alpha, double beta, bool enabled = true);

  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_ddepth(enum DDEPTH v);
  enum DDEPTH ddepth() const;

  void set_alpha(double  v);
  double alpha() const;

  void set_beta(double  v);
  double beta() const;

  void set_auto_scale(bool v);
  bool auto_scale() const;

protected:
  double alpha_ = 1.0;
  double beta_ = 0.0;
  DDEPTH ddepth_ = DDEPTH_32F;
  bool auto_scale_ = true;
};


std::string toStdString(enum c_type_convert_routine::DDEPTH v);
enum c_type_convert_routine::DDEPTH fromStdString(const std::string & s,
    enum c_type_convert_routine::DDEPTH defval);

#endif /* __c_type_convert_routine_h__ */

