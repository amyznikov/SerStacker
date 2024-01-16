/*
 * c_sharpness_norm_measure.cc
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#include "c_sharpness_norm_measure.h"
#include <core/proc/unsharp_mask.h>
#include <core/ssprintf.h>

//template<>
//const c_enum_member* members_of<c_sharpness_norm_measure::cv::NormTypes>()
//{
//  static const c_enum_member members[] = {
//      { c_sharpness_norm_measure::NORM_L1, "NORM_L1", "cv::NORM_L1" },
//      { c_sharpness_norm_measure::NORM_L2, "NORM_L2", "cv::NORM_L2" },
//      { c_sharpness_norm_measure::NORM_L2SQR, "NORM_L2SQR", "cv::NORM_L2SQR" },
//      { c_sharpness_norm_measure::NORM_INF, "NORM_INF", "cv::NORM_INF" },
//      { c_sharpness_norm_measure::NORM_L1 }
//  };
//
//  return members;
//}

void c_sharpness_norm_measure::set_norm_type(cv::NormTypes v)
{
  norm_type_ = v;
}

cv::NormTypes c_sharpness_norm_measure::norm_type() const
{
  return norm_type_;
}

double c_sharpness_norm_measure::sigma() const
{
  return sigma_;
}

void c_sharpness_norm_measure::set_sigma(double v)
{
  sigma_ = v;
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask,
    double sigma, cv::NormTypes norm_type)
{
  return hpass_norm(src, sigma, mask, norm_type);
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask) const
{
  return measure(src, mask, sigma_, norm_type_);
}

double  c_sharpness_norm_measure::add(cv::InputArray src, cv::InputArray mask)
{
  const double v =
      measure(src, mask);

  accumulator_ += v;
  counter_ += 1;

  return v;
}

double c_sharpness_norm_measure::average() const
{
  return accumulator_ / counter_;
}

void c_sharpness_norm_measure::reset()
{
  accumulator_ = 0;
  counter_ = 0;
}

double c_sharpness_norm_measure::accumulator() const
{
  return accumulator_;
}

int c_sharpness_norm_measure::counter() const
{
  return counter_;
}
