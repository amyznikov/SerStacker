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

      { color_channel_max_absdiff , "max_absdiff",  "max(abs(image))"},
      { color_channel_first_nonzero, "first_nonzero", "first nonzero value"},

      { color_channel_max_color, "max_color", "max - min"},
      { color_channel_max_gradient, "max_gradient", "max gradient"},

      { color_channel_unknown}
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

    scaled_src =
        src.getMat();

    scaled_mask =
        srcmsk.getMat();
  }
  else if ( output_scale == 0.5 ) {

    cv::pyrDown(src, scaled_src);

    if ( !srcmsk.empty() ) {
      cv::pyrDown(srcmsk, scaled_mask,
          scaled_src.size());
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
        break;

      case color_channel_luminance_YCrCb:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to YCrCb not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2YCrCb);
        cv::extractChannel(converted_src, converted_src, 0);
        break;

      case color_channel_luminance_lab:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to Lab not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2Lab);
        cv::extractChannel(converted_src, converted_src, 0);
        if( converted_src.depth() == CV_32F || converted_src.depth() == CV_64F ) {
          cv::multiply(converted_src, 1e-2, converted_src);
        }
        break;

      case color_channel_luminance_luv:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to Luv not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2Luv);
        cv::extractChannel(converted_src, converted_src, 0);
        if( converted_src.depth() == CV_32F || converted_src.depth() == CV_64F ) {
          cv::multiply(converted_src, 1e-2, converted_src);
        }
        break;

      case color_channel_luminance_hsv:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to HSV not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2HSV);
        cv::extractChannel(converted_src, converted_src, 2);
        break;

      case color_channel_luminance_hls:
        if( scaled_src.channels() != 3 ) {
          CF_ERROR("Invalid argument: conversion to HLS not supported for image with %d channels",
              scaled_src.channels());
          return false;
        }
        cv::cvtColor(scaled_src, converted_src, cv::COLOR_BGR2HLS);
        cv::extractChannel(converted_src, converted_src, 1);
        break;

      case color_channel_blue:
        cv::extractChannel(scaled_src, converted_src, 0);
        break;


      case color_channel_green:
        if( scaled_src.channels() < 2 ) {
          CF_ERROR("Invalid argument: Requested channel index 'green' (1) is out of range of input image channels %d",
              scaled_src.channels());
          return false;
        }
        cv::extractChannel(scaled_src, converted_src, 1);
        break;


      case color_channel_red:
        if( scaled_src.channels() < 3 ) {
          CF_ERROR("Invalid argument: Requested channel index 'red' (2) is out of range of input image channels %d",
              scaled_src.channels());
          return false;
        }
        cv::extractChannel(scaled_src, converted_src, 2);
        break;


      case color_channel_min_inensity: {
        if( scaled_src.isContinuous() ) {
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


      case color_channel_max_intensity: {
        if( scaled_src.isContinuous() ) {
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

      case color_channel_avg_intensity: {
        if( scaled_src.isContinuous() ) {
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

      case color_channel_sum_intensity: {
        if( scaled_src.isContinuous() ) {
          const int src_rows = scaled_src.rows;
          cv::reduce(scaled_src.reshape(1, scaled_src.total()), converted_src, 1, cv::REDUCE_SUM);
          converted_src = converted_src.reshape(0, src_rows);
        }
        else {
          cv::Mat tmp = scaled_src.clone();
          const int src_rows = tmp.rows;
          cv::reduce(tmp.reshape(1, tmp.total()), converted_src, 1, cv::REDUCE_SUM);
          converted_src = converted_src.reshape(0, src_rows);
        }
        break;
      }


      case color_channel_max_absdiff: {

        std::vector<cv::Mat> src_channels;
        cv::Mat tmp1, tmp2, mask;

        cv::split(scaled_src, src_channels);

        src_channels[0].copyTo(converted_src);
        tmp1 = cv::abs(converted_src);

        for( int c = 1, cn = src_channels.size(); c < cn; ++c ) {

          tmp2 = cv::abs(src_channels[c]);

          cv::compare(tmp2, tmp1, mask, cv::CMP_GT);

          src_channels[c].copyTo(converted_src, mask);

          if( c < cn - 1 ) {
            tmp2.copyTo(tmp1, mask);
          }
        }

        break;
      }

      case color_channel_first_nonzero: {

        std::vector<cv::Mat> src_channels;
        cv::Mat m1, m2;

        cv::split(scaled_src, src_channels);

        src_channels[0].copyTo(converted_src);
        cv::compare(src_channels[0], 0, m1, cv::CMP_NE);

        for( int c = 1, cn = src_channels.size(); c < cn; ++c ) {

          cv::compare(src_channels[c], 0, m2, cv::CMP_NE);

          src_channels[c].copyTo(converted_src, m2 & m1);

          if( c < cn - 1 ) {
            m1.setTo(0, m2);
          }

        }

        break;
      }



      case color_channel_max_color: {

        const cv::Mat tmp =
            scaled_src.isContinuous() ? scaled_src :
                scaled_src.clone();

        const int src_rows =
            scaled_src.rows;

        cv::Mat cmin, cmax;

        cv::reduce(tmp.reshape(1, tmp.total()), cmin, 1, cv::REDUCE_MIN);
        cv::reduce(tmp.reshape(1, tmp.total()), cmax, 1, cv::REDUCE_MAX);
        cv::subtract(cmax, cmin, converted_src);
        converted_src = converted_src.reshape(0, src_rows);

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

//        src_channels[0].copyTo(converted_src);
//        gchannels[0].copyTo(g);
//
//        for( int c = 1, cn = gchannels.size(); c < cn; ++c ) {
//          cv::compare(gchannels[c], g, gm, cv::CMP_GT);
//          gchannels[c].copyTo(g, gm);
//          src_channels[c].copyTo(converted_src, gm);
//        }

        break;
      }

      default:
        CF_ERROR("Invalid color channel requested: %d",
            channel);
        return false;
    }
  }

  if( output_depth < 0 ) {

    output_depth =
        dst.fixedType() ? dst.depth() :
            src.depth();
  }


  if( output_depth == src.depth() && output_depth_scale == 1.0 ) {

    if( converted_src.empty() ) {
      scaled_src.copyTo(dst);
    }
    else {
      dst.move(converted_src);
    }

  }
  else if( converted_src.empty() ) {
    scaled_src.convertTo(dst, output_depth,
        output_depth_scale);
  }
  else {
    converted_src.convertTo(dst, output_depth,
        output_depth_scale);
  }

  if( dstmsk.needed() ) {
    scaled_mask.copyTo(dstmsk);
  }

  return true;
}
