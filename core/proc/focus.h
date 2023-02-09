/*
 * focus.h
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __focus_h__
#define __focus_h__

#include "sharpness_measure/c_local_contrast_measure.h"
#include "sharpness_measure/c_lpg_sharpness_measure.h"
#include "sharpness_measure/c_harris_sharpness_measure.h"
#include "sharpness_measure/c_normalized_variance_measure.h"
#include "sharpness_measure/c_sharpness_norm_measure.h"

enum SHARPNESS_MEASURE {
  SHARPNESS_MEASURE_LCM = 0,
  SHARPNESS_MEASURE_LPG = 1,
  SHARPNESS_MEASURE_HARRIS,
  SHARPNESS_MEASURE_NORMALIZED_VARIANCE,
  SHARPNESS_MEASURE_SHARPNESS_NORM,
};

class c_camera_focus_measure
{
public:
  typedef c_camera_focus_measure this_class;

  void set_method(enum SHARPNESS_MEASURE v);
  enum SHARPNESS_MEASURE method() const;

  //  void set_avgchannel(bool v);
  bool avgchannel() const;

  c_local_contrast_measure & local_contrast_measure();
  const c_local_contrast_measure & local_contrast_measure() const;

  c_lpg_sharpness_measure & lpg_measure();
  const c_lpg_sharpness_measure & lpg_measure() const;

  c_harris_sharpness_measure & harris_measure();
  const c_harris_sharpness_measure harris_measure() const;

  c_normalized_variance_measure & normalized_variance_measure();
  const c_normalized_variance_measure & normalized_variance_measure() const;

  c_sharpness_norm_measure & sharpness_norm_measure();
  const c_sharpness_norm_measure & sharpness_norm_measure() const;

  cv::Scalar measure(cv::InputArray image) const;

protected:
  SHARPNESS_MEASURE method_ = SHARPNESS_MEASURE_LCM;
  //bool avgchannel_ = false;

  c_local_contrast_measure local_contrast_measure_;
  c_lpg_sharpness_measure lpg_measure_;
  c_harris_sharpness_measure harris_measure_;
  c_normalized_variance_measure normalized_variance_measure_;
  c_sharpness_norm_measure sharpness_norm_measure_;
};

#endif /* __focus_h__ */

