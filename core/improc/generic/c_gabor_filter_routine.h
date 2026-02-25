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
    _ksize = v;
    _update_filter_bank = true;
  }

  const cv::Size & ksize() const
  {
    return _ksize;
  }

  void set_sigma(double v)
  {
    _sigma = v;
    _update_filter_bank = true;
  }

  double sigma() const
  {
    return _sigma;
  }

  void set_min_theta(double v)
  {
    _min_theta = v;
    _update_filter_bank = true;
  }

  double min_theta() const
  {
    return _min_theta;
  }

  void set_max_theta(double v)
  {
    _max_theta = v;
    _update_filter_bank = true;
  }

  double max_theta() const
  {
    return _max_theta;
  }

  void set_num_theta(int v)
  {
    _num_theta = v;
    _update_filter_bank = true;
  }

  int num_theta() const
  {
    return _num_theta;
  }

  void set_lambd(double v)
  {
    _lambd = v;
    _update_filter_bank = true;
  }

  double lambd() const
  {
    return _lambd;
  }

  void set_gamma(double v)
  {
    _gamma = v;
    _update_filter_bank = true;
  }

  double gamma() const
  {
    return _gamma;
  }

  void set_psi(double v)
  {
    _psi = v;
    _update_filter_bank = true;
  }

  double psi() const
  {
    return _psi;
  }

  void set_ktype(PIXEL_DEPTH v)
  {
    _ktype = v;
    _update_filter_bank = true;
  }

  PIXEL_DEPTH ktype() const
  {
    return _ktype;
  }

  void set_display_kernels(bool v)
  {
    _display_kernels = v;
    _update_filter_bank = true;
  }

  bool display_kernels() const
  {
    return _display_kernels;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Size _ksize = cv::Size(15, 15);
  double _sigma = 2.5; // [px]
  double _min_theta = 0;// [deg]
  double _max_theta = 360; // [deg]
  int _num_theta = 8;
  double _lambd = 1.0; // [px]
  double _gamma = 3; // [px]
  double _psi = 90; // [deg]
  PIXEL_DEPTH _ktype = PIXEL_DEPTH_32F;
  bool _display_kernels = false;
  bool _update_filter_bank = true;

  std::vector<cv::Mat> _filters;

};

#endif /* __c_gabor_filter_routine_h__ */
