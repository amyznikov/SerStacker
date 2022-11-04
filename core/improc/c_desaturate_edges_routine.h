/*
 * c_desaturate_edges_routine.h
 *
 *  Created on: Oct 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_desaturate_edges_routine_h__
#define __c_desaturate_edges_routine_h__

#include "c_image_processor.h"

class c_desaturate_edges_routine:
    public c_image_processor_routine
{
public:
  typedef c_desaturate_edges_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("desaturate_edges", "desaturate_edges", "desaturate planetary dsk edges",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_desaturate_edges_routine(bool enabled = true);
  c_desaturate_edges_routine(double w, bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double weight, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  bool compute_planetary_disk_weights(const cv::Mat & src_ecc_image,
      const cv::Mat & src_mask,
      cv::Mat1f & weights) const;

  void set_alpha(double v);
  double alpha() const;

  void set_gbsigma(double v);
  double gbsigma() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_blur_radius(double v);
  double blur_radius() const;

  void set_l1norm(bool v);
  bool l1norm() const;

  void set_show_weights(bool v);
  bool show_weights() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, alpha, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, gbsigma, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stdev_factor, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, blur_radius, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, l1norm, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, show_weights, "");
  }


protected:
  double alpha_ = 0.5;
  double gbsigma_ = 1;
  double stdev_factor_ = 0.25;
  double blur_radius_ = 1;
  bool show_weights_ = false;
  bool l1norm_ = false;
};

#endif /* __c_desaturate_edges_routine_h__ */
