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
      "desaturate_edges", "Desaturate color on planetary disk edges");

  void set_alpha(double v);
  double alpha() const;

  void set_gbsigma(double v);
  double gbsigma() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_se_close_radius(int v);
  int se_close_radius() const;

  void set_blur_radius(double v);
  double blur_radius() const;

  void set_l1norm(bool v);
  bool l1norm() const;

  void set_show_weights(bool v);
  bool show_weights() const;

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

  bool compute_planetary_disk_weights(const cv::Mat & src_ecc_image,
      const cv::Mat & src_mask,
      cv::Mat1f & weights) const;

protected:
  double alpha_ = 0.5;
  double gbsigma_ = 1;
  double stdev_factor_ = 0.25;
  double blur_radius_ = 1;
  int se_close_radius_ = 2;
  bool show_weights_ = false;
  bool l1norm_ = false;
};

#endif /* __c_desaturate_edges_routine_h__ */
