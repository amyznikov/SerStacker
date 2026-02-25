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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

  bool compute_planetary_disk_weights(const cv::Mat & src_ecc_image,
      const cv::Mat & src_mask,
      cv::Mat1f & weights) const;

protected:
  double _alpha = 0.5;
  double _gbsigma = 1;
  double _stdev_factor = 0.25;
  double _blur_radius = 1;
  int _se_close_radius = 2;
  bool _show_weights = false;
  bool _l1norm = false;
};

#endif /* __c_desaturate_edges_routine_h__ */
