/*
 * c_neighbourhood_average_routine.h
 *
 *  Created on: Nov 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_neighbourhood_average_routine_h__
#define __c_neighbourhood_average_routine_h__

#include <core/improc/c_image_processor.h>

class c_neighbourhood_average_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_neighbourhood_average_routine,
      "c_neighbourhood_average_routine", "cv::filter2d with hole in center of SE");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::MorphShapes _se_shape = cv::MORPH_RECT;
  cv::BorderTypes _border_type = cv::BORDER_REPLICATE;
  cv::Size _se_size = cv::Size(3, 3);
  cv::Point _anchor = cv::Point(-1, -1);
  //cv::Scalar _border_value;
};

#endif /* __c_neighbourhood_average_routine_h__ */
