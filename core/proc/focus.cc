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
      { SHARPNESS_MEASURE_LCM, "LCM", "local contrast map from c_local_contrast_measure" },
      { SHARPNESS_MEASURE_LPG, "LPG", "laplacian + gradient from c_lpg_sharpness_measure" },
      { SHARPNESS_MEASURE_MGMAP, "MGMAP", "Morphological gradient gradient from c_mgmap_sharpness_measure" },
      { SHARPNESS_MEASURE_NORMALIZED_VARIANCE, "NORMALIZED_VARIANCE", "Normalized variance stdtev(image)/mean(image)" },
      { SHARPNESS_MEASURE_HARRIS, "HARRIS", "FocusALL: Focal Stacking of Microscopic Images Using Modified Harris Corner Response Measure" },
      { SHARPNESS_MEASURE_SHARPNESS_NORM, "SHARPNESS_NORM", "from c_sharpness_norm_measure class" },

      { SHARPNESS_MEASURE_LPG },
  };

  return members;
}

void c_camera_focus_measure::set_method(enum SHARPNESS_MEASURE v)
{
  _method = v;
}

enum SHARPNESS_MEASURE c_camera_focus_measure::method() const
{
  return _method;
}

//void c_camera_focus_measure::set_avgchannel(bool v)
//{
//  avgchannel_ = v;
//}

bool c_camera_focus_measure::avgchannel() const
{
  switch (_method) {
    case SHARPNESS_MEASURE_LCM:
      return _local_contrast_measure.avgchannel();

    case SHARPNESS_MEASURE_LPG:
      return true; // lpg_measure_.avgchannel();

    case SHARPNESS_MEASURE_HARRIS:
      return _harris_measure.avgchannel();

    case SHARPNESS_MEASURE_NORMALIZED_VARIANCE:
      return _normalized_variance_measure.avgchannel();

    case SHARPNESS_MEASURE_SHARPNESS_NORM:
      return true; // sharpness_norm_measure_.avgchannel();
  }

  return false;
}

c_local_contrast_measure & c_camera_focus_measure::local_contrast_measure()
{
  return _local_contrast_measure;
}

const c_local_contrast_measure & c_camera_focus_measure::local_contrast_measure() const
{
  return _local_contrast_measure;
}

c_lpg_sharpness_measure & c_camera_focus_measure::lpg_measure()
{
  return _lpg_measure;
}

const c_lpg_sharpness_measure & c_camera_focus_measure::lpg_measure() const
{
  return _lpg_measure;
}

c_harris_sharpness_measure & c_camera_focus_measure::harris_measure()
{
  return _harris_measure;
}

const c_harris_sharpness_measure c_camera_focus_measure::harris_measure() const
{
  return _harris_measure;
}

c_normalized_variance_measure & c_camera_focus_measure::normalized_variance_measure()
{
  return _normalized_variance_measure;
}

const c_normalized_variance_measure & c_camera_focus_measure::normalized_variance_measure() const
{
  return _normalized_variance_measure;
}

c_sharpness_norm_measure & c_camera_focus_measure::sharpness_norm_measure()
{
  return _sharpness_norm_measure;
}

const c_sharpness_norm_measure & c_camera_focus_measure::sharpness_norm_measure() const
{
  return _sharpness_norm_measure;
}

cv::Scalar c_camera_focus_measure::measure(cv::InputArray image) const
{
  switch (_method) {
    case SHARPNESS_MEASURE_LCM:
      return _local_contrast_measure.compute(image);

    case SHARPNESS_MEASURE_LPG:
      return _lpg_measure.compute(image);

    case SHARPNESS_MEASURE_MGMAP:
      return _mgmap_measure.compute(image);

    case SHARPNESS_MEASURE_HARRIS:
      return _harris_measure.compute(image);

    case SHARPNESS_MEASURE_NORMALIZED_VARIANCE:
      return _normalized_variance_measure.compute(image);

    case SHARPNESS_MEASURE_SHARPNESS_NORM:
      return cv::Scalar::all(_sharpness_norm_measure.measure(image));
  }

  return cv::Scalar::all(0);
}

