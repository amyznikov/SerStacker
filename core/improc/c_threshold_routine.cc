/*
 * c_threshold_routine.cc
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#include "c_threshold_routine.h"
#include <core/proc/threshold.h>
#include <core/proc/reduce_channels.h>

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

bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int cmpop = -1;

  switch (threshold_type_) {
    case THRESHOLD_TYPE_CMP_GT:
      cmpop = cv::CMP_GT;
      break;
    case THRESHOLD_TYPE_CMP_GE:
      cmpop = cv::CMP_GE;
      break;
    case THRESHOLD_TYPE_CMP_LT:
      cmpop = cv::CMP_LT;
      break;
    case THRESHOLD_TYPE_CMP_LE:
      cmpop = cv::CMP_LE;
      break;
    case THRESHOLD_TYPE_CMP_EQ:
      cmpop = cv::CMP_EQ;
      break;
    case THRESHOLD_TYPE_CMP_NE:
      cmpop = cv::CMP_NE;
      break;
    default:
      break;
  }

  if( cmpop >= 0 ) {
    if( !modify_mask_ ) {
      cv::compare(image.getMat(), threshold_value_, image, cmpop);
    }
    else if( mask.empty() ) {
      cv::compare(image.getMat(), threshold_value_, mask, cmpop);
      if( mask.channels() > 1 ) {
        reduce_color_channels(mask, mask, cv::REDUCE_MIN);
      }
    }
    else {
      cv::Mat m;
      cv::compare(image.getMat(), threshold_value_, m, cmpop);
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

  std::vector<cv::Mat> channels;
  cv::Mat M;

  if( image.channels() == 1 ) {
    channels = image.getMat();
  }
  else {
    cv::split(image, channels);
  }

  for( int i = 0, n = channels.size(); i < n; ++i ) {

    double threshold_value = 0;

    switch (threshold_type_) {
      case THRESHOLD_TYPE_OTSU:
        threshold_value = get_otsu_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_TRIANGLE:
        threshold_value = get_triangle_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_MOMENTS:
        threshold_value = get_moments_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_ISODATA:
        threshold_value = get_isodata_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_HUANG:
        threshold_value = get_huang_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_YEN:
        threshold_value = get_yen_threshold(channels[i], mask);
        break;
      case THRESHOLD_TYPE_MEAN:
        threshold_value = cv::mean(channels[i], mask)[0];
        break;
      case THRESHOLD_TYPE_MINIMUM:
        threshold_value = get_minimum_threshold(channels[i], mask);
        break;
      default:
        continue;
    }

    cv::compare(channels[i], threshold_value, channels[i], cv::CMP_GT);
  }


  if( channels.size() == 1 ) {
    M = channels[0];
  }
  else {
    cv::merge(channels, M);
  }

  if( !modify_mask_ ) {
    M.convertTo(image, image.depth());
  }
  else if( mask.empty() ) {
    if( M.channels() > 1 ) {
      reduce_color_channels(M, mask, cv::REDUCE_MIN);
    }
    else {
      mask.move(M);
    }
  }
  else {
    if( mask.channels() > 1 ) {
      reduce_color_channels(mask, mask, cv::REDUCE_MIN);
    }
    if( M.channels() > 1 ) {
      reduce_color_channels(M, M, cv::REDUCE_MIN);
    }
    cv::bitwise_and(M, mask, mask);
  }

  return true;
}

