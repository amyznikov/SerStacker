/*
 * c_ecc_options.h
 *
 *  Created on: Jun 1, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ecc_options_h__
#define __c_ecc_options_h__

#include <core/proc/eccalign.h>

struct c_ecc_options {
  double scale = 1.;
  double eps = 0.2;
  double min_rho = 0.8;
  double input_smooth_sigma = 1.0;
  double reference_smooth_sigma = 1.0;
  double update_step_scale = 1.5;
  double normalization_noise = 0.01;
  int normalization_scale = 0;
  int max_iterations = 15;
  int ecch_minimum_image_size = 32;
  bool enable_ecch = false;
};

struct c_eccflow_options {
  double update_multiplier = 1.5;
  double input_smooth_sigma = 0;
  double reference_smooth_sigma = 0;
  int max_iterations = 1;
  int support_scale = 4;
  int normalization_scale = -1;
};


#endif /* __c_ecc_options_h__ */
