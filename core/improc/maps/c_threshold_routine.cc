/*
 * c_threshold_routine.cc
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#include "c_threshold_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>


template<>
const c_enum_member* members_of<cv::CmpTypes>()
{
  static constexpr c_enum_member members[] = {
      { cv::CMP_EQ, "EQ", "src is equal to value" },
      { cv::CMP_GT, "GT", "src is greater than value" },
      { cv::CMP_GE, "GE", "src is greater than or equal to  value" },
      { cv::CMP_LT, "LT", "src is less than  value" },
      { cv::CMP_LE, "LE", "src is less than or equal to  value" },
      { cv::CMP_NE, "NE", "src is not equal to  value" },
      { cv::CMP_GT },
  };

  return members;
}

bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  std::vector<cv::Mat> channels;

  const int cn =
      image.channels();

  if( cn == 1 ) {
    channels.emplace_back(image.getMat());
  }
  else {
    cv::split(image, channels);
  }

  for ( int i = 0; i < cn; ++i ) {

    const double threshold_value =
        get_threshold_value(channels[i], mask,
            threshold_type_,
            threshold_value_);

    cv::compare(channels[i], threshold_value, channels[i],
        compare_);
  }

  if( !modify_mask_ ) {
    if( cn == 1 ) {
      image.move(channels[0]);
    }
    else {
      cv::merge(channels, image);
    }
  }
  else if( mask.empty() ) {
    if( cn == 1 ) {
      mask.move(channels[0]);
    }
    else {
      cv::merge(channels, mask);
      reduce_color_channels(mask, mask, cv::REDUCE_MIN);
    }
  }
  else {
    if( mask.channels() > 1 ) {
      reduce_color_channels(mask, mask, cv::REDUCE_MIN);
    }
    if( cn > 1 ) {
      for( int i = 1; i < cn; ++i ) {
        cv::bitwise_and(channels[0], channels[i], channels[0]);
      }
    }
    cv::bitwise_and(channels[0], mask, mask);
  }

  return true;
}

