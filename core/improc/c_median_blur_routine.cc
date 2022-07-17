/*
 * c_image_processor_routine.cc
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#include "c_median_blur_routine.h"
#include <core/proc/normalize.h>

c_median_blur_routine::c_class_factory c_median_blur_routine::class_factory;

template<>
const c_enum_member * members_of<cv::ximgproc::WMFWeightType>()
{
  using namespace cv::ximgproc;

  static constexpr c_enum_member members[] = {
      {WMF_OFF, "OFF", "unweighted"},
      {WMF_EXP, "EXP", "exp(-|I1-I2|^2/(2*sigma^2))"},
      {WMF_IV1, "IV1", "(|I1-I2|+sigma)^-1"},
      {WMF_IV2, "IV2", "(|I1-I2|^2+sigma^2)^-1"},
      {WMF_COS, "COS", "dot(I1,I2)/(|I1|*|I2|)"},
      {WMF_JAC, "JAC", "(min(r1,r2)+min(g1,g2)+min(b1,b2))/(max(r1,r2)+max(g1,g2)+max(b1,b2))"},
      {WMF_OFF}, // must be last
  };

  return members;
}


c_median_blur_routine::c_median_blur_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_median_blur_routine::ptr c_median_blur_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_median_blur_routine::set_radius(int v)
{
  radius_ = v;
}

int c_median_blur_routine::radius() const
{
  return radius_;
}

void c_median_blur_routine::set_sigma(double v)
{
  sigma_ = v;
}

double c_median_blur_routine::sigma() const
{
  return sigma_;
}

void c_median_blur_routine::set_weightType(cv::ximgproc::WMFWeightType v)
{
  weightType_ = v;
}

cv::ximgproc::WMFWeightType c_median_blur_routine::weightType() const
{
  return weightType_;
}

void c_median_blur_routine::set_ignore_mask(bool v)
{
  ignore_mask_ = v;
}

bool c_median_blur_routine::ignore_mask() const
{
  return ignore_mask_;
}


bool c_median_blur_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, weightType);
  LOAD_PROPERTY(settings, this, radius);
  LOAD_PROPERTY(settings, this, sigma);
  LOAD_PROPERTY(settings, this, ignore_mask);

  return true;
}

bool c_median_blur_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, weightType);
  SAVE_PROPERTY(settings, *this, radius);
  SAVE_PROPERTY(settings, *this, sigma);
  SAVE_PROPERTY(settings, *this, ignore_mask);

  return true;
}

bool c_median_blur_routine::process(cv::InputOutputArray _image, cv::InputOutputArray _mask)
{
  using namespace cv::ximgproc;

  cv::Mat joint;
  cv::Mat mask;

  if ( !ignore_mask_ ) {
    mask = _mask.getMat();
  }

  if( weightType_ == WMF_OFF ) {
    joint = cv::Mat1b::ones(_image.size());
  }
  else {
    switch (_image.depth()) {
    case CV_8U:
      case CV_8S:
      joint = _image.getMatRef();
      break;
    default:
      normalize_meanStdDev(_image.getMat(), joint, 6, 0, 255, _mask, cv::BORDER_CONSTANT);
      joint.convertTo(joint, CV_8U);
      break;
    }
  }


 weightedMedianFilter(joint, _image, _image, radius_, sigma_, weightType_, mask);

  return true;
}
