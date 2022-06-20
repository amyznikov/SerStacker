/*
 * c_histogram_normalization_routine.cc
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#include "c_histogram_normalization_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_histogram_normalization_routine::histogram_normalization_type>()
{
  static constexpr c_enum_member members[] = {
      { c_histogram_normalization_routine::normalize_mean, "mean" },
      //{ c_histogram_normalization_routine::normalize_median, "median" },
      { c_histogram_normalization_routine::normalize_mean, nullptr, }  // must  be last
  };

  return members;
}


c_histogram_normalization_routine::c_class_factory c_histogram_normalization_routine::class_factory;



c_histogram_normalization_routine::c_histogram_normalization_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_histogram_normalization_routine::ptr c_histogram_normalization_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_histogram_normalization_routine::set_normalization_type(histogram_normalization_type v)
{
  normalization_type_ = v;
}

c_histogram_normalization_routine::histogram_normalization_type c_histogram_normalization_routine::normalization_type() const
{
  return normalization_type_;
}

void c_histogram_normalization_routine::set_offset(const cv::Scalar & v)
{
  offset_ = v;
}

const cv::Scalar & c_histogram_normalization_routine::offset() const
{
  return offset_;
}

void c_histogram_normalization_routine::set_scale(const cv::Scalar & v)
{
  scale_ = v;
}

const cv::Scalar & c_histogram_normalization_routine::scale() const
{
  return scale_;
}

bool c_histogram_normalization_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, normalization_type);
  LOAD_PROPERTY(settings, this, offset);
  LOAD_PROPERTY(settings, this, scale);

  return true;
}

bool c_histogram_normalization_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, normalization_type);
  SAVE_PROPERTY(settings, *this, offset);
  SAVE_PROPERTY(settings, *this, scale);

  return true;
}

bool c_histogram_normalization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    const cv::Scalar m =
        cv::mean(image, mask);

    cv::add(image, offset_ - m, image, mask,
        image.depth());
  }

  return true;
}
