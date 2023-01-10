/*
 * c_equalize_hist_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_equalize_hist_routine_h__
#define __c_equalize_hist_routine_h__

#include "c_image_processor.h"

class c_equalize_hist_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_equalize_hist_routine,
      "equalize_hist", "calls cv::equalizeHist()");
};

#endif /* __c_equalize_hist_routine_h__ */
