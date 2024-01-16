/*
 * focus.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "focus.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<SHARPNESS_MEASURE>()
{
  static const c_enum_member members[] = {
      { SHARPNESS_MEASURE_LCM, "LCM", "local contrast map from c_local_contrast_measure class" },
      { SHARPNESS_MEASURE_LPG, "LPG", "laplacian + gradient from c_lpg_sharpness_measure class" },
      { SHARPNESS_MEASURE_NORMALIZED_VARIANCE, "NORMALIZED_VARIANCE", "Normalized variance stdtev(image)/mean(image)" },
      { SHARPNESS_MEASURE_HARRIS, "HARRIS", "FocusALL: Focal Stacking of Microscopic Images Using Modified Harris Corner Response Measure" },
      { SHARPNESS_MEASURE_SHARPNESS_NORM, "SHARPNESS_NORM", "from c_sharpness_norm_measure class" },

      { SHARPNESS_MEASURE_LPG },
  };

  return members;
}

void c_camera_focus_measure::set_method(enum SHARPNESS_MEASURE v)
{
  method_ = v;
}

enum SHARPNESS_MEASURE c_camera_focus_measure::method() const
{
  return method_;
}

//void c_camera_focus_measure::set_avgchannel(bool v)
//{
//  avgchannel_ = v;
//}

bool c_camera_focus_measure::avgchannel() const
{
  switch (method_) {
    case SHARPNESS_MEASURE_LCM:
      return local_contrast_measure_.avgchannel();

    case SHARPNESS_MEASURE_LPG:
      return lpg_measure_.avgchannel();

    case SHARPNESS_MEASURE_HARRIS:
      return harris_measure_.avgchannel();

    case SHARPNESS_MEASURE_NORMALIZED_VARIANCE:
      return normalized_variance_measure_.avgchannel();

    case SHARPNESS_MEASURE_SHARPNESS_NORM:
      return true; // sharpness_norm_measure_.avgchannel();
  }

  return false;
}

c_local_contrast_measure & c_camera_focus_measure::local_contrast_measure()
{
  return local_contrast_measure_;
}

const c_local_contrast_measure & c_camera_focus_measure::local_contrast_measure() const
{
  return local_contrast_measure_;
}

c_lpg_sharpness_measure & c_camera_focus_measure::lpg_measure()
{
  return lpg_measure_;
}

const c_lpg_sharpness_measure & c_camera_focus_measure::lpg_measure() const
{
  return lpg_measure_;
}

c_harris_sharpness_measure & c_camera_focus_measure::harris_measure()
{
  return harris_measure_;
}

const c_harris_sharpness_measure c_camera_focus_measure::harris_measure() const
{
  return harris_measure_;
}

c_normalized_variance_measure & c_camera_focus_measure::normalized_variance_measure()
{
  return normalized_variance_measure_;
}

const c_normalized_variance_measure & c_camera_focus_measure::normalized_variance_measure() const
{
  return normalized_variance_measure_;
}

c_sharpness_norm_measure & c_camera_focus_measure::sharpness_norm_measure()
{
  return sharpness_norm_measure_;
}

const c_sharpness_norm_measure & c_camera_focus_measure::sharpness_norm_measure() const
{
  return sharpness_norm_measure_;
}

cv::Scalar c_camera_focus_measure::measure(cv::InputArray image) const
{
  switch (method_) {
    case SHARPNESS_MEASURE_LCM:
      return local_contrast_measure_.compute(image);

    case SHARPNESS_MEASURE_LPG:
      return lpg_measure_.compute(image);

    case SHARPNESS_MEASURE_HARRIS:
      return harris_measure_.compute(image);

    case SHARPNESS_MEASURE_NORMALIZED_VARIANCE:
      return normalized_variance_measure_.compute(image);

    case SHARPNESS_MEASURE_SHARPNESS_NORM:
      return cv::Scalar::all(sharpness_norm_measure_.measure(image));
  }

  return cv::Scalar::all(0);
}

