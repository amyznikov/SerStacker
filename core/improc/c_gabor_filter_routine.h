/*
 * c_gabor_filter_routine.h
 *
 *  Created on: May 2, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gabor_filter_routine_h__
#define __c_gabor_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_gabor_filter_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gabor_filter_routine,
      "gabor_filter",
      "Apply Gabor filter bank to image.\n"
      "f(x,y) = exp(-0.5 * (x/sigma)^2 - 0.5 * (y*gamma/sigma)^2 ) * cos(2 * PI *x / lambd + psi)\n");

  void set_ksize(const cv::Size & v)
  {
    ksize_ = v;
    update_filter_bank_ = true;
  }

  const cv::Size & ksize() const
  {
    return ksize_;
  }

  void set_sigma(double v)
  {
    sigma_ = v;
    update_filter_bank_ = true;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_min_theta(double v)
  {
    min_theta_ = v;
    update_filter_bank_ = true;
  }

  double min_theta() const
  {
    return min_theta_;
  }

  void set_max_theta(double v)
  {
    max_theta_ = v;
    update_filter_bank_ = true;
  }

  double max_theta() const
  {
    return max_theta_;
  }

  void set_num_theta(int v)
  {
    num_theta_ = v;
    update_filter_bank_ = true;
  }

  int num_theta() const
  {
    return num_theta_;
  }

  void set_lambd(double v)
  {
    lambd_ = v;
    update_filter_bank_ = true;
  }

  double lambd() const
  {
    return lambd_;
  }

  void set_gamma(double v)
  {
    gamma_ = v;
    update_filter_bank_ = true;
  }

  double gamma() const
  {
    return gamma_;
  }

  void set_psi(double v)
  {
    psi_ = v;
    update_filter_bank_ = true;
  }

  double psi() const
  {
    return psi_;
  }

  void set_ktype(PIXEL_DEPTH v)
  {
    ktype_ = v;
    update_filter_bank_ = true;
  }

  PIXEL_DEPTH ktype() const
  {
    return ktype_;
  }

  void set_display_kernels(bool v)
  {
    display_kernels_ = v;
    update_filter_bank_ = true;
  }

  bool display_kernels() const
  {
    return display_kernels_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Size ksize_ = cv::Size(15, 15);
  double sigma_ = 2.5; // [px]
  double min_theta_ = 0;// [deg]
  double max_theta_ = 360; // [deg]
  int num_theta_ = 8;
  double lambd_ = 1.0; // [px]
  double gamma_ = 3; // [px]
  double psi_ = 90; // [deg]
  PIXEL_DEPTH ktype_ = PIXEL_DEPTH_32F;
  bool display_kernels_ = false;
  bool update_filter_bank_ = true;

  std::vector<cv::Mat> filters_;

};

#endif /* __c_gabor_filter_routine_h__ */
