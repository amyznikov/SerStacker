/*
 * c_threshold_routine.cc
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#include "c_threshold_routine.h"
#include <core/proc/threshold.h>
#include <core/ssprintf.h>


template<>
const c_enum_member* members_of<THRESHOLD_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { THRESHOLD_TYPE_CMP_GT, "GT", "" },
      { THRESHOLD_TYPE_CMP_GE, "GE", "" },
      { THRESHOLD_TYPE_CMP_LT, "LT", "" },
      { THRESHOLD_TYPE_CMP_LE, "LE", "" },
      { THRESHOLD_TYPE_CMP_EQ, "EQ", "" },
      { THRESHOLD_TYPE_CMP_NE, "NE", "" },

      { THRESHOLD_TYPE_OTSU, "OTSU", "OTSU" },
      { THRESHOLD_TYPE_TRIANGLE, "TRIANGLE", "TRIANGLE" },
      { THRESHOLD_TYPE_MOMENTS, "MOMENTS", "MOMENTS" },
      { THRESHOLD_TYPE_ISODATA, "ISODATA", "ISODATA" },
      { THRESHOLD_TYPE_HUANG, "HUANG", "HUANG" },
      { THRESHOLD_TYPE_YEN, "YEN", "YEN" },
      { THRESHOLD_TYPE_MEAN, "MEAN", "MEAN" },
      { THRESHOLD_TYPE_MINIMUM, "MINIMUM", "MINIMUM" },
      { THRESHOLD_TYPE_OTSU }
  };

  return members;
}


c_threshold_routine::c_class_factory c_threshold_routine::class_factory;

c_threshold_routine::c_threshold_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_threshold_routine::ptr c_threshold_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  bool handled = true;

  switch (threshold_type_) {
    case THRESHOLD_TYPE_CMP_GT:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_CMP_GE:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_GE);
      break;
    case THRESHOLD_TYPE_CMP_LT:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_LT);
      break;
    case THRESHOLD_TYPE_CMP_LE:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_LE);
      break;
    case THRESHOLD_TYPE_CMP_EQ:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_EQ);
      break;
    case THRESHOLD_TYPE_CMP_NE:
      cv::compare(image.getMat(), threshold_value_, image, cv::CMP_NE);
      break;
    default:
      handled = false;
      break;
  }

  if ( handled ) {
    return true;
  }


  std::vector<cv::Mat> channels;

  if ( image.channels() == 1 ) {
    channels = image.getMat();
  }
  else {
    cv::split(image, channels);
  }

  for( int i = 0, n = channels.size(); i < n; ++i ) {
    switch (threshold_type_) {
    case THRESHOLD_TYPE_OTSU:
      cv::compare(channels[i], get_otsu_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_TRIANGLE:
      cv::compare(channels[i], get_triangle_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_MOMENTS:
      cv::compare(channels[i], get_moments_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_ISODATA:
      cv::compare(channels[i], get_isodata_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_HUANG:
      cv::compare(channels[i], get_huang_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_YEN:
      cv::compare(channels[i], get_yen_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_MEAN:
      cv::compare(channels[i], cv::mean(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    case THRESHOLD_TYPE_MINIMUM:
      cv::compare(channels[i], get_minimum_threshold(channels[i], mask), channels[i], cv::CMP_GT);
      break;
    default:
      break;
    }
  }

  if ( channels.size() == 1 ) {
    channels[0].convertTo(image, image.depth());
  }
  else {
    cv::Mat T;
    cv::merge(channels, T);
    T.convertTo(image, image.depth());
  }


  return true;
}

bool c_threshold_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, threshold_type);
  LOAD_PROPERTY(settings, this, threshold_value);

  return true;
}

bool c_threshold_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, threshold_type);
  SAVE_PROPERTY(settings, *this, threshold_value);

  return true;
}


void c_threshold_routine::set_threshold_type(THRESHOLD_TYPE v)
{
  threshold_type_ = v;
}

THRESHOLD_TYPE c_threshold_routine::threshold_type() const
{
  return threshold_type_;
}

void c_threshold_routine::set_threshold_value(double v)
{
  threshold_value_ = v;
}

double c_threshold_routine::threshold_value() const
{
  return threshold_value_;
}
