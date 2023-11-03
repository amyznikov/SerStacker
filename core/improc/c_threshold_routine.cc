/*
 * c_threshold_routine.cc
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#include "c_threshold_routine.h"
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>





template<>
const c_enum_member* members_of<c_threshold_routine::THRESHOLD_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { c_threshold_routine::THRESHOLD_VALUE, "VALUE", "Use user-specified value for compare operation"},
      { c_threshold_routine::THRESHOLD_OTSU, "OTSU", "Use Otsu algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_TRIANGLE, "TRIANGLE", "Use Triangle algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_MOMENTS, "MOMENTS", "Use MOMENTS algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_ISODATA, "ISODATA", "Use ISODATA algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_HUANG, "HUANG", "Use HUANG algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_YEN, "YEN", "Use YEN algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_MEAN, "MEAN", "Select pixels with values above mean value" },
      { c_threshold_routine::THRESHOLD_MINIMUM, "MINIMUM", "Use MINIMUM algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_PLANETARY_DISK, "PLANETARY_DISK", "" },
      { c_threshold_routine::THRESHOLD_NOISE, "NOISE", "" },
      { c_threshold_routine::THRESHOLD_CLEAR, "CLEAR", "" },

      { c_threshold_routine::THRESHOLD_OTSU }
  };

  return members;
}


bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( threshold_type_ == THRESHOLD_CLEAR ) {
    if( modify_mask_ ) {
      mask.release();
    }
    else {
      image.getMatRef().setTo(255);
    }
    return true;
  }


  const int cn =
      image.channels();

  cv::Mat m;

  if ( threshold_type_ == THRESHOLD_PLANETARY_DISK ) {

    bool fOK =
        simple_planetary_disk_detector(image, mask,
            nullptr,
            1,
            0.25 * threshold_scale_,
            nullptr,
            &m,
            nullptr,
            nullptr);

    if ( !fOK ) {
      CF_ERROR("simple_planetary_disk_detector() fails");
      return false;
    }

    if ( fill_holes_ ) {
      geo_fill_holes(m, m, 8);
    }

    switch (compare_) {
      case cv::CMP_GT:
        break;
      case cv::CMP_LT:
        cv::bitwise_not(m, m);
        break;
      case cv::CMP_EQ:
        morphological_gradient(m, m);
        break;
      case cv::CMP_NE:
        morphological_gradient(m, m);
        cv::bitwise_not(m, m);
        break;
      case cv::CMP_GE:
        morphological_internal_gradient(m, m);
        break;
      case cv::CMP_LE:
        morphological_external_gradient(m, m);
        break;
    }
  }
  else {

    std::vector<cv::Mat> channels;

    if( cn == 1 ) {
      channels.emplace_back(image.getMat().clone());
    }
    else {
      cv::split(image, channels);
    }

    for ( int i = 0; i < cn; ++i ) {

      const double threshold_value =
          get_threshold_value(channels[i], mask,
              (::THRESHOLD_TYPE)threshold_type_,
              threshold_value_);

      cv::compare(channels[i], threshold_value * threshold_scale_, channels[i],
          compare_);

      if ( !modify_mask_ && fill_holes_ ) {
        geo_fill_holes(channels[i], channels[i], 8);
      }

    }

    if( cn == 1 ) {
      m = channels[0];
    }
    else {
      cv::merge(channels, m);
    }

    if ( modify_mask_ && fill_holes_ ) {
      geo_fill_holes(m, m, 8);
    }

  }

  if ( invert_ ) {
    cv::bitwise_not(m, m);
  }

  if( !modify_mask_ ) {
    image.move(m);
  }
  else if( mask.empty() ) {

    if( m.channels() == 1 ) {
      mask.move(m);
    }
    else {
      reduce_color_channels(m, mask, cv::REDUCE_MIN);
    }
  }
  else {
    if( mask.channels() > 1 ) {
      reduce_color_channels(mask, mask, cv::REDUCE_MIN);
    }
    if( m.channels() > 1 ) {
      reduce_color_channels(m, m, cv::REDUCE_MIN);
    }


    cv::bitwise_and(m, mask, mask);

  }


  return true;
}

