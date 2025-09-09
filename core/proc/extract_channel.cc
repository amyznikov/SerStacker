/*
 * extract_channel.cc
 *
 *  Created on: Jan 14, 2020
 *      Author: amyznikov
 */

#include "extract_channel.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<color_channel_type>()
{
  static const c_enum_member members[] = {
      { color_channel_0, "0", "cv::extractChannel(0)" },
      { color_channel_1, "1", "cv::extractChannel(1)" },
      { color_channel_2, "2", "cv::extractChannel(2)" },
      { color_channel_3, "3", "cv::extractChannel(3)" },
      { color_channel_4, "4", "cv::extractChannel(4)" },

      { color_channel_dont_change, "dont_change", "Don't change color channels"},

      { color_channel_red, "red", "cv::extractChannel(2)"},
      { color_channel_green, "green", "cv::extractChannel(1)" },
      { color_channel_blue, "blue", "cv::extractChannel(0)"},

      { color_channel_gray, "gray", "cv::cvtColor(cv::COLOR_BGR2GRAY)"},
      { color_channel_luminance_YCrCb, "luminance_YCrCb", "cv::cvtColor(cv::COLOR_BGR2YCRCB) -> cv::extractChannel(0)"},
      { color_channel_luminance_lab, "luminance_lab", "cv::cvtColor(cv::COLOR_BGR2LAB)->cv::extractChannel(0)"},
      { color_channel_luminance_luv, "luminance_luv", "cv::cvtColor(cv::COLOR_BGR2Luv)->cv::extractChannel(0)"},
      { color_channel_luminance_hsv, "luminance_hsv", "cv::cvtColor(cv::COLOR_BGR2HSV)->cv::extractChannel(2)"},
      { color_channel_luminance_hls, "luminance_hls", "cv::cvtColor(cv::COLOR_BGR2HLS)->cv::extractChannel(0)"},


      { color_channel_min_inensity, "min", "cv::reduce(cv::REDUCE_MIN)"},
      { color_channel_max_intensity, "max", "cv::reduce(cv::REDUCE_MAX)"},
      { color_channel_avg_intensity, "avg", "cv::reduce(cv::REDUCE_AVG)"},
      { color_channel_sum_intensity, "sum", "cv::reduce(cv::REDUCE_SUM)"},

      { color_channel_absmax , "absmax",  "max of absolute pixel value "},
      { color_channel_first_nonzero, "first_nonzero", "first nonzero value"},

      { color_channel_max_color, "max_color", "max - min"},
      { color_channel_max_gradient, "max_gradient", "max gradient"},

      { color_channel_unknown}
  };
  return members;
}


template<class T>
static void extract_first_non_zero_channel_(cv::InputArray _src, cv::OutputArray _dst)
{
  const int rows =
      _src.rows();

  const int cols =
      _src.cols();

  const int channels =
      _src.channels();

  const cv::Mat_<T> src =
      _src.getMat();

  _dst.create(rows, cols, CV_MAKETYPE(src.depth(), 1));

  cv::Mat_<T> dst =
      _dst.getMatRef();

  for( int y = 0; y < rows; ++y ) {

    const T * srcp = src[y];
    T * dstp = dst[y];

    for( int x = 0; x < cols; ++x ) {

      int c = 0;
      while ( c < channels && !(dstp[x] = srcp[x * channels + c]) ) {
        ++c;
      }
    }
  }
}

static void extract_first_non_zero_channel(cv::InputArray src, cv::OutputArray dst)
{
  switch (src.depth()) {
    case CV_8U:
      return extract_first_non_zero_channel_<uint8_t>(src, dst);
    case CV_8S:
      return extract_first_non_zero_channel_<int8_t>(src, dst);
    case CV_16U:
      return extract_first_non_zero_channel_<uint16_t>(src, dst);
    case CV_16S:
      return extract_first_non_zero_channel_<int16_t>(src, dst);
    case CV_32S:
      return extract_first_non_zero_channel_<int32_t>(src, dst);
    case CV_32F:
      return extract_first_non_zero_channel_<float>(src, dst);
    case CV_64F:
      return extract_first_non_zero_channel_<double>(src, dst);
  }
}



template<class T>
static void extract_absmax_channel_(cv::InputArray _src, cv::OutputArray _dst)
{
  const int rows =
      _src.rows();

  const int cols =
      _src.cols();

  const int channels =
      _src.channels();

  const cv::Mat_<T> src =
      _src.getMat();

  _dst.create(rows, cols, CV_MAKETYPE(src.depth(), 1));

  cv::Mat_<T> dst =
      _dst.getMatRef();

  for( int y = 0; y < rows; ++y ) {

    const T * srcp = src[y];
    T * dstp = dst[y];

    for( int x = 0; x < cols; ++x ) {

      T v1 =
          std::abs(dstp[x] = srcp[x * channels + 0]);

      for( int c = 1; c < channels; ++c ) {

        const T v2 =
            std::abs(srcp[x * channels + c]);

        if( v2 > v1 ) {
          dstp[x] = srcp[x * channels + c];
          v1 = v2;
        }
      }

    }
  }

}

static void extract_absmax_channel(cv::InputArray src, cv::OutputArray dst)
{
  switch (src.depth()) {
    case CV_8U:
      return extract_absmax_channel_<uint8_t>(src, dst);
    case CV_8S:
      return extract_absmax_channel_<int8_t>(src, dst);
    case CV_16U:
      return extract_absmax_channel_<uint16_t>(src, dst);
    case CV_16S:
      return extract_absmax_channel_<int16_t>(src, dst);
    case CV_32S:
      return extract_absmax_channel_<int32_t>(src, dst);
    case CV_32F:
      return extract_absmax_channel_<float>(src, dst);
    case CV_64F:
      return extract_absmax_channel_<double>(src, dst);
  }
}


static void convert_image(cv::Mat & image, int dst_depth)
{
  if ( image.depth() != dst_depth ) {

    double scale = 1;
    double offset = 0;

    get_scale_offset(image.depth(), dst_depth,
        &scale, &offset);

    image.convertTo(image, dst_depth,
        scale, offset);
  }

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


  if( output_depth < 0 ) {

    output_depth =
        dst.fixedType() ? dst.depth() :
            src.depth();
  }

  if (output_scale <= 0 || std::abs(output_scale - 1.) < FLT_EPSILON) {

    scaled_src =
        src.getMat();

    scaled_mask =
        srcmsk.getMat();
  }
  else if ( std::abs(output_scale - 0.5) < FLT_EPSILON ) {

    cv::pyrDown(src, scaled_src);

    if ( !srcmsk.empty() ) {

      cv::pyrDown(srcmsk, scaled_mask,
          scaled_src.size());

      cv::compare(scaled_mask, cv::Scalar::all(250),
          scaled_mask,
          cv::CMP_GE);
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
          cv::INTER_AREA);

      cv::compare(scaled_mask, cv::Scalar::all(250),
          scaled_mask,
          cv::CMP_GE);

    }
  }

  if ( channel == color_channel_dont_change ) {
    converted_src =
        scaled_src;
  }
  else if ( channel < color_channel_featured_begin ) {

    if ( channel < 0 || channel >= scaled_src.channels() ) {
      CF_ERROR("Invalid argument: requested channel %d is out of range src.channels()=%d",
          channel, scaled_src.channels());
      return false;
    }

    if ( scaled_src.channels() > 0 ) {
      cv::extractChannel(scaled_src, scaled_src,
          channel);
    }

    if( !scaled_mask.empty() && scaled_mask.channels() > channel ) {
      cv::extractChannel(scaled_mask, scaled_mask,
          channel);
    }

  }

  else if( scaled_src.channels() > 1 ) {

    switch (channel) {

      case color_channel_gray:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to gray not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }

        cv::cvtColor(scaled_src, converted_src,
            cv::COLOR_BGR2GRAY);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }

        break;

      case color_channel_luminance_YCrCb:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to YCrCb not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2YCrCb);
        cv::extractChannel(converted_src, converted_src, 0);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }
        break;

      case color_channel_luminance_lab: {

        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to Lab not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }

        const int src_depth =
            scaled_src.depth();

        const bool conversion_required =
            src_depth != CV_8U && src_depth != CV_32F;

        if( conversion_required ) {
          convert_image(scaled_src, CV_32F);
        }

        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2Lab);
        cv::extractChannel(converted_src, converted_src, 0);
        if( converted_src.depth() == CV_32F || converted_src.depth() == CV_64F ) {
          cv::multiply(converted_src, 1e-2, converted_src);
        }

        convert_image(converted_src,
            output_depth);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }
        break;
      }

      case color_channel_luminance_luv: {

        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to Luv not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }

        const int src_depth =
            scaled_src.depth();

        const bool conversion_required =
            src_depth != CV_8U && src_depth != CV_32F;

        if( conversion_required ) {
          convert_image(scaled_src, CV_32F);
        }


        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2Luv);
        cv::extractChannel(converted_src, converted_src, 0);
        if( converted_src.depth() == CV_32F || converted_src.depth() == CV_64F ) {
          cv::multiply(converted_src, 1e-2, converted_src);
        }

        convert_image(converted_src,
            output_depth);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }
        break;
      }

      case color_channel_luminance_hsv: {
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to HSV not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }

        const int src_depth =
            scaled_src.depth();

        const bool conversion_required =
            src_depth != CV_8U && src_depth != CV_32F;

        if( conversion_required ) {
          convert_image(scaled_src, CV_32F);
        }

        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2HSV);
        cv::extractChannel(converted_src, converted_src, 2);

        convert_image(converted_src,
            output_depth);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }
        break;
      }

      case color_channel_luminance_hls: {

        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to HLS not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }

        const int src_depth =
            scaled_src.depth();

        const bool conversion_required =
            src_depth != CV_8U && src_depth != CV_32F;

        if( conversion_required ) {
          convert_image(scaled_src, CV_32F);
        }

        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2HLS);
        cv::extractChannel(converted_src, converted_src, 1);

        convert_image(converted_src,
            output_depth);

        if( !scaled_mask.empty() && scaled_mask.channels() != 1 ) {
          reduce_color_channels(scaled_mask, cv::REDUCE_MAX);
        }
        break;
      }

      case color_channel_blue:

        cv::extractChannel(scaled_src, converted_src, 0);

        if( !scaled_mask.empty() && scaled_mask.channels() > 0 ) {
          cv::extractChannel(scaled_mask, scaled_mask, 0);
        }
        break;


      case color_channel_green:
        if( scaled_src.channels() < 2 ) {
          CF_ERROR("Invalid argument: Requested channel index 'green' (1) is out of range of input image channels %d",
              scaled_src.channels());
          return false;
        }

        cv::extractChannel(scaled_src, converted_src, 1);

        if( !scaled_mask.empty() && scaled_mask.channels() > 1 ) {
          cv::extractChannel(scaled_mask, scaled_mask, 1);
        }
        break;


      case color_channel_red:
        if( scaled_src.channels() < 3 ) {
          CF_ERROR("Invalid argument: Requested channel index 'red' (2) is out of range of input image channels %d",
              scaled_src.channels());
          return false;
        }

        cv::extractChannel(scaled_src, converted_src, 2);

        if( !scaled_mask.empty() && scaled_mask.channels() > 2 ) {
          cv::extractChannel(scaled_mask, scaled_mask, 2);
        }
        break;


      case color_channel_min_inensity: {
        reduce_color_channels(scaled_src, converted_src,
            cv::REDUCE_MIN);
        break;
      }


      case color_channel_max_intensity: {
        reduce_color_channels(scaled_src, converted_src,
            cv::REDUCE_MAX);
        break;
      }

      case color_channel_avg_intensity: {

        const int src_depth =
            scaled_src.depth();

        const bool conversion_required =
            src_depth != CV_8U && src_depth != CV_32F;

        if( conversion_required ) {
          convert_image(scaled_src, CV_32F);
        }

        reduce_color_channels(scaled_src, converted_src,
            cv::REDUCE_AVG);

        convert_image(converted_src,
            output_depth);

        break;
      }

      case color_channel_sum_intensity: {

        if( scaled_src.depth() != CV_32F ) {
          convert_image(scaled_src, CV_32F);
        }

        reduce_color_channels(scaled_src, converted_src,
            cv::REDUCE_SUM);

        convert_image(converted_src,
            output_depth);

        break;
      }


      case color_channel_absmax: {
        extract_absmax_channel(scaled_src,
            converted_src);
        break;
      }

      case color_channel_first_nonzero: {
        extract_first_non_zero_channel(scaled_src,
            converted_src);
        break;
      }


      case color_channel_max_color: {

        cv::Mat cmin, cmax;

        reduce_color_channels(scaled_src, cmin,
            cv::REDUCE_MIN);

        reduce_color_channels(scaled_src, cmax,
            cv::REDUCE_MAX);

        cv::subtract(cmax, cmin, converted_src);

        break;
      }

      case color_channel_max_gradient : {

        static const auto compute_sobel_gradients =
            [](cv::InputArray src, cv::OutputArray g, int ddepth, int borderType) -> void {

              static thread_local cv::Mat Kx, Ky;
              if( Kx.empty() ) {
                cv::getDerivKernels(Kx, Ky, 1, 0, 5, true, CV_32F);
                Kx *= M_SQRT2;
                Ky *= M_SQRT2;
              }

              if( ddepth < 0 ) {
                ddepth = std::max(src.depth(), CV_32F);
              }

              cv::Mat gx, gy;

              cv::sepFilter2D(src, gx, ddepth, Kx, Ky, cv::Point(-1, -1), 0, borderType);
              cv::sepFilter2D(src, gy, ddepth, Ky, Kx, cv::Point(-1, -1), 0, borderType);
              cv::magnitude(gx, gy, g);
            };


        cv::Mat g, gm;
        std::vector<cv::Mat> gchannels;
        std::vector<cv::Mat> src_channels;
        double min, max;

        compute_sobel_gradients(scaled_src, g, -1, cv::BORDER_REPLICATE);

        cv::split(g, gchannels);
        cv::split(scaled_src, src_channels);

        converted_src.release();

        for( int c = 0, cn = gchannels.size(); c < cn; ++c ) {

          cv::minMaxLoc(gm, &min, &max);
          cv::add(gchannels[c], max > min ? 1e-3 * (max - min) : 1e-3, gchannels[c]);

          if( converted_src.empty() ) {
            cv::multiply(src_channels[c], gchannels[c],
                converted_src);
          }
          else {
            cv::multiply(src_channels[c], gchannels[c], src_channels[c]);
            cv::add(src_channels[c], converted_src, converted_src);
          }

          if( gm.empty() ) {
            gchannels[c].copyTo(gm);
          }
          else {
            cv::add(gchannels[c], gm, gm);
          }
        }

        cv::divide(converted_src, gm, converted_src);


        break;
      }

      default:
        CF_ERROR("Invalid color channel requested: %d",
            channel);
        return false;
    }
  }

  if( converted_src.empty() ) {

    if( output_depth == src.depth() && output_depth_scale == 1.0 ) {
      scaled_src.copyTo(dst);
    }
    else {
      scaled_src.convertTo(dst, output_depth,
          output_depth_scale);
    }

  }
  else {

    if( output_depth == converted_src.depth() && output_depth_scale == 1.0 ) {
      dst.move(converted_src);
    }
    else {
      converted_src.convertTo(dst, output_depth,
          output_depth_scale);
    }
  }


  if( dstmsk.needed() ) {

    if( scaled_mask.empty() || scaled_mask.channels() == 1 ) {
      dstmsk.move(scaled_mask);
    }
    else {
      reduce_color_channels(scaled_mask, dstmsk, cv::REDUCE_MAX);
    }
  }

  return true;
}
