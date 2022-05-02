/*
 * c_mtf_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mtf_routine_h__
#define __c_mtf_routine_h__

#include <core/mtf/c_pixinsight_mtf.h>
#include "c_image_processor.h"

class c_mtf_routine
    : public c_image_processor_routine
{
public:
  typedef c_mtf_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("mtf", "mtf", "mtf",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_mtf_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  c_pixinsight_mtf & mtf();

  const c_pixinsight_mtf & mtf() const;

  void set_shadows(double v) {
    mtf_.set_shadows(v);
  }

  double shadows() const {
    return mtf_.shadows();
  }

  void set_highlights(double v) {
    mtf_.set_highlights(v);
  }

  double highlights() const {
    return mtf_.highlights();
  }

  void set_midtones(double v) {
    mtf_.set_midtones(v);
  }

  double midtones() const {
    return mtf_.midtones();
  }


protected:
  c_pixinsight_mtf mtf_;
};

#endif /* __c_mtf_routine_h__ */
