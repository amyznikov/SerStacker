/*
 * c_seed_fill_segmentation_routine.h
 *
 *  Created on: May 5, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_seed_fill_segmentation_routine_h__
#define __c_seed_fill_segmentation_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/seed_fill_segmentation.h>

class c_seed_fill_segmentation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_seed_fill_segmentation_routine, "seed_fill_segmentation",
      "C++/OpenCV implementation of Connected component labeling algorithm from paper by M. Emre Celebi"
      "'A Simple and Efficient Algorithm for Connected Component Labeling in Color Images'");

  void set_threshold(double v)
  {
    _threshold = v;
  }

  double threshold() const
  {
    return _threshold;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double _threshold = 1;
};

#endif /* __c_seed_fill_segmentation_routine_h__ */
