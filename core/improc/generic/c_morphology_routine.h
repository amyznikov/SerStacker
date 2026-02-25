/*
 * c_morphology_routine.h
 *
 *  Created on: Jun 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_
#define CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_

#include <core/improc/c_image_processor.h>
#include <core/proc/morphology.h>
//#include <core/proc/geo-reconstruction.h>


class c_morphology_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_morphology_routine,
      "morphology",
      "Apply morphological operation on image");

  void set_se_shape(cv::MorphShapes v)
  {
    _se_shape = v;
    SE.release();
  }

  cv::MorphShapes se_shape() const
  {
    return _se_shape;
  }

  void set_se_size(const cv::Size & v)
  {
    _se_size = v;
    SE.release();
  }

  const cv::Size& se_size() const
  {
    return _se_size;
  }

  void set_anchor(const cv::Point & v)
  {
    _anchor = v;
    SE.release();
  }

  const cv::Point & anchor() const
  {
    return _anchor;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::MorphShapes _se_shape = cv::MORPH_RECT;
  cv::Size _se_size = cv::Size(3, 3);
  cv::Point _anchor = cv::Point(-1,-1);
  MORPH_OPERATION _operation = MORPH_ERODE;
  int _iterations = 1;
  cv::BorderTypes _borderType = cv::BORDER_CONSTANT;
  cv::Scalar _borderValue = cv::Scalar::all(0);
  cv::Mat SE;
};

#endif /* CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_ */
