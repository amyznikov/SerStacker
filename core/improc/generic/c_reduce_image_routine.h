/*
 * c_reduce_image_routine.h
 *
 *  Created on: Jun 25, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_reduce_image_routine_h__
#define __c_reduce_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_reduce_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_reduce_image_routine,
      "reduce_image", "");

  enum REDUCE_DIM {
    REDUCE_ROWS = 0, // 0 means that matrix is reduced to a single row
    REDUCE_COLS = 1, // 1 means that the matrix is reduced to a single column
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Rect _rect;
  REDUCE_DIM _dim = REDUCE_COLS;
  cv::ReduceTypes _rtype = cv::REDUCE_AVG;
  bool _useROISelection = false;

};

#endif /* __c_reduce_image_routine_h__ */
