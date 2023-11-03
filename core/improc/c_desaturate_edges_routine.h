/*
 * c_desaturate_edges_routine.h
 *
 *  Created on: Oct 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_desaturate_edges_routine_h__
#define __c_desaturate_edges_routine_h__

#include <core/improc/c_image_processor.h>

class c_desaturate_edges_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_desaturate_edges_routine,
      "desaturate_edges", "Desaturate color on planetary dsk edges");

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

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, alpha);
      SERIALIZE_PROPERTY(settings, save, *this, gbsigma);
      SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
      SERIALIZE_PROPERTY(settings, save, *this, blur_radius);
      SERIALIZE_PROPERTY(settings, save, *this, l1norm);
      SERIALIZE_PROPERTY(settings, save, *this, show_weights);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  bool compute_planetary_disk_weights(const cv::Mat & src_ecc_image,
      const cv::Mat & src_mask,
      cv::Mat1f & weights) const;

protected:
  double alpha_ = 0.5;
  double gbsigma_ = 1;
  double stdev_factor_ = 0.25;
  double blur_radius_ = 1;
  bool show_weights_ = false;
  bool l1norm_ = false;
};

#endif /* __c_desaturate_edges_routine_h__ */
