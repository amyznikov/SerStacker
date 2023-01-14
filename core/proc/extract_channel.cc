/*
 * extract_channel.cc
 *
 *  Created on: Jan 14, 2020
 *      Author: amyznikov
 */

#include "extract_channel.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<color_channel_type>()
{
  static constexpr c_enum_member members[] = {
      { color_channel_0, "0", "" },
      { color_channel_1, "1", "" },
      { color_channel_2, "2", "" },
      { color_channel_3, "3", "" },
      { color_channel_4, "4", "" },

      { color_channel_gray, "gray", },
      { color_channel_luminance, "luminance (Lab)", },
      { color_channel_red, "red", },
      { color_channel_green, "green", },
      { color_channel_blue, "blue", },
      { color_channel_min_inensity, "min_inensity", },
      { color_channel_max_inensity, "max_inensity", },
      { color_channel_avg_inensity, "avg_inensity", },

      { color_channel_unknown, nullptr, }
  };
  return members;
}

// Extract requested color channel form input color image
bool extract_channel(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray srcmsk, cv::OutputArray dstmsk,
    int channel,
    double output_scale,
    int output_depth,
    double output_depth_scale)
{
  cv::Mat scaled_src, converted_src;
  cv::Mat scaled_mask;

  if ( src.channels() < 1 ) {
    CF_ERROR("Invalid argument: channel extraction not supported for image with %d channels",
        src.channels());
    return false;
  }


  if ( output_scale <= 0 || output_scale == 1 ) {
    scaled_src = src.getMat();
    scaled_mask = srcmsk.getMat();
  }
  else if ( output_scale == 0.5 ) {
    cv::pyrDown(src, scaled_src);
    if ( !srcmsk.empty() ) {
      cv::pyrDown(srcmsk, scaled_mask, scaled_src.size());
    }
  }
  else {

    cv::resize(src, scaled_src, cv::Size(0, 0),
        output_scale,
        output_scale,
        cv::INTER_AREA);

    if ( !srcmsk.empty() ) {
      cv::resize(srcmsk, scaled_mask, cv::Size(0, 0),
          output_scale,
          output_scale,
          cv::INTER_NEAREST);
    }
  }

  if ( channel < color_channel_featured_begin ) {
    if ( channel < 0 || channel >= scaled_src.channels() ) {
      CF_ERROR("Invalid argument: requested channel %d is out of range src.channels()=%d",
          channel, scaled_src.channels());
      return false;
    }
    if ( scaled_src.channels() > 0 ) {
      cv::extractChannel(scaled_src, scaled_src, channel);
    }
  }
  else if ( scaled_src.channels() > 1 ) {
    switch ( channel ) {
    case color_channel_gray :
      if ( scaled_src.channels() != 3 ) {
        CF_ERROR("Invalid argument: conversion to gray not supported for image with %d channels",
            scaled_src.channels());
        return false;
      }

      cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2GRAY);
      break;

    case color_channel_luminance :
      if ( scaled_src.channels() != 3 ) {
        CF_ERROR("Invalid argument: conversion to Lab not supported for image with %d channels",
            scaled_src.channels());
        return false;
      }
      cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2Lab);
      cv::extractChannel(converted_src, converted_src, 0);
      if ( converted_src.depth() == CV_32F || converted_src.depth() == CV_64F ) {
        cv::multiply(converted_src, 1e-2, converted_src);
      }
      break;

    case color_channel_blue:
      cv::extractChannel(scaled_src, converted_src, 0);
      break;

    case color_channel_green :
      if ( scaled_src.channels() < 2 ) {
        CF_ERROR("Invalid argument: Requested channel index 'green' (1) is out of range of input image channels %d",
            scaled_src.channels());
        return false;
      }
      cv::extractChannel(scaled_src, converted_src, 1);
      break;

    case color_channel_red :
      if ( scaled_src.channels() < 3 ) {
        CF_ERROR("Invalid argument: Requested channel index 'red' (2) is out of range of input image channels %d",
            scaled_src.channels());
        return false;
      }
      cv::extractChannel(scaled_src, converted_src, 2);
      break;

    case color_channel_min_inensity : {
      if ( scaled_src.isContinuous() ) {
        const int src_rows = scaled_src.rows;
        cv::reduce(scaled_src.reshape(1, scaled_src.total()), converted_src, 1, cv::REDUCE_MIN);
        converted_src = converted_src.reshape(0, src_rows);
      }
      else {
        cv::Mat tmp = scaled_src.clone();
        const int src_rows = tmp.rows;
        cv::reduce(tmp.reshape(1, tmp.total()), converted_src, 1, cv::REDUCE_MIN);
        converted_src = converted_src.reshape(0, src_rows);
      }
      break;
    }

    case color_channel_max_inensity : {
      if ( scaled_src.isContinuous() ) {
        const int src_rows = scaled_src.rows;
        cv::reduce(scaled_src.reshape(1, scaled_src.total()), converted_src, 1, cv::REDUCE_MAX);
        converted_src = converted_src.reshape(0, src_rows);
      }
      else {
        cv::Mat tmp = scaled_src.clone();
        const int src_rows = tmp.rows;
        cv::reduce(tmp.reshape(1, tmp.total()), converted_src, 1, cv::REDUCE_MAX);
        converted_src = converted_src.reshape(0, src_rows);
      }
      break;
    }

    case color_channel_avg_inensity : {
      if ( scaled_src.isContinuous() ) {
        const int src_rows = scaled_src.rows;
        cv::reduce(scaled_src.reshape(1, scaled_src.total()), converted_src, 1, cv::REDUCE_AVG);
        converted_src = converted_src.reshape(0, src_rows);
      }
      else {
        cv::Mat tmp = scaled_src.clone();
        const int src_rows = tmp.rows;
        cv::reduce(tmp.reshape(1, tmp.total()), converted_src, 1, cv::REDUCE_AVG);
        converted_src = converted_src.reshape(0, src_rows);
      }
      break;
    }

    default:
      CF_ERROR("Invalid color channel requested: %d",
          channel);
      return false;
    }
  }

  if ( output_depth < 0 ) {
    if ( dst.fixedType() ) {
      output_depth = dst.depth();
    }
    else {
      output_depth = src.depth();
    }
  }

  if ( output_depth == src.depth() && output_depth_scale == 1.0 ) {
    if ( converted_src.empty() ) {
      scaled_src.copyTo(dst);
    }
    else {
      dst.move(converted_src);
    }
  }
  else {
    if ( converted_src.empty() ) {
      scaled_src.convertTo(dst, output_depth, output_depth_scale);
    }
    else {
      converted_src.convertTo(dst, output_depth, output_depth_scale);
    }
  }

  if ( dstmsk.needed() ) {
    scaled_mask.copyTo(dstmsk);
  }

  return true;

}
