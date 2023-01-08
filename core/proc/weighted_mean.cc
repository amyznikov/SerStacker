/*
 * weighted_mean.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "weighted_mean.h"
#include <core/debug.h>


template<class T1, class T2>
static cv::Scalar weighted_mean_(const cv::Mat & src, const cv::Mat & weights)
{
  const cv::Size src_size =
      src.size();

  const cv::Size weights_size =
      weights.size();

  if( src_size != weights_size ) {
    CF_ERROR("ERROR: src_size=%dx%d not equal to weights_size=%dx%d",
        src_size.width, src_size.height,
        weights_size.width, weights_size.height);
    return cv::Scalar::all(0);
  }

  const int src_channels =
      src.channels();

  const int weights_channels =
      weights.channels();

  if( src_channels != weights_channels && weights_channels != 1 ) {
    CF_ERROR("ERROR: invalid number of channels: src_channels=%d weigts_channels=%d",
        src_channels, weights_channels);
    return cv::Scalar::all(0);
  }

  cv::Scalar total =
      cv::Scalar::all(0);

  const cv::Scalar w =
      cv::sum(weights);

  if ( src_channels == weights_channels ) {

    for( int y = 0; y < src_size.height; ++y ) {

      const T1 *srcp =
          src.ptr<const T1>(y);

      const T2 * wp =
          weights.ptr<const T2>(y);

      for( int x = 0; x < src_size.width; ++x ) {

        for( int c = 0; c < src_channels; ++c ) {
          total[c] +=
              srcp[x * src_channels + c] * wp[x * src_channels + c];
        }
      }
    }

    for( int c = 0; c < src_channels; ++c ) {
      total[c] /= w[c];
    }

  }
  else {

    for( int y = 0; y < src_size.height; ++y ) {

      const T1 *srcp =
          src.ptr<const T1>(y);

      const T2 * wp =
          weights.ptr<const T2>(y);

      for( int x = 0; x < src_size.width; ++x ) {

        for( int c = 0; c < src_channels; ++c ) {
          total[c] +=
              srcp[x * src_channels + c] * wp[x];
        }
      }
    }

    for( int c = 0; c < src_channels; ++c ) {
      total[c] /= w[0];
    }
  }


  return total;
}


cv::Scalar weighted_mean(cv::InputArray _src, cv::InputArray _weights)
{
  const cv::Mat src =
      _src.getMat();

  const cv::Mat weights =
      _weights.getMat();

  switch (src.depth()) {
    case CV_8U:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<uint8_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<uint8_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<uint8_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<uint8_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<uint8_t, uint32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<uint8_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<uint8_t, double>(src, weights);
      }
      break;
    case CV_8S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int8_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int8_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int8_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int8_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int8_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int8_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int8_t, double>(src, weights);
      }
      break;
    case CV_16U:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<uint16_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<uint16_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<uint16_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<uint16_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<uint16_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<uint16_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<uint16_t, double>(src, weights);
      }
      break;
    case CV_16S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int16_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int16_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int16_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int16_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int16_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int16_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int16_t, double>(src, weights);
      }
      break;
    case CV_32S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int32_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int32_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int32_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int32_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int32_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int32_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int32_t, double>(src, weights);
      }
      break;
    case CV_32F:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<float, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<float, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<float, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<float, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<float, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<float, float>(src, weights);
        case CV_64F:
          return weighted_mean_<float, double>(src, weights);
      }
      break;
    case CV_64F:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<double, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<double, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<double, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<double, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<double, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<double, float>(src, weights);
        case CV_64F:
          return weighted_mean_<double, double>(src, weights);
      }
      break;
  }

  return cv::Scalar::all(0);
}



