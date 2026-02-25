/*
 * c_sharpness_norm_measure.cc
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#include "c_sharpness_norm_measure.h"
#include <core/proc/unsharp_mask.h>
#include <core/ssprintf.h>

void c_sharpness_norm_measure::set_norm_type(cv::NormTypes v)
{
  _opts.norm_type = v;
}

cv::NormTypes c_sharpness_norm_measure::norm_type() const
{
  return _opts.norm_type;
}

double c_sharpness_norm_measure::sigma() const
{
  return _opts.sigma;
}

void c_sharpness_norm_measure::set_sigma(double v)
{
  _opts.sigma = v;
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask,
    double sigma, cv::NormTypes norm_type)
{
  return hpass_norm(src, sigma, mask, norm_type);
}

double c_sharpness_norm_measure::measure(cv::InputArray src, cv::InputArray mask) const
{
  return measure(src, mask, _opts.sigma, _opts.norm_type);
}

double  c_sharpness_norm_measure::add(cv::InputArray src, cv::InputArray mask)
{
  const double v =
      measure(src, mask);

  _accumulator += v;
  _counter += 1;

  return v;
}

double c_sharpness_norm_measure::average() const
{
  return _accumulator / _counter;
}

void c_sharpness_norm_measure::reset()
{
  _accumulator = 0;
  _counter = 0;
}

double c_sharpness_norm_measure::accumulator() const
{
  return _accumulator;
}

int c_sharpness_norm_measure::counter() const
{
  return _counter;
}
