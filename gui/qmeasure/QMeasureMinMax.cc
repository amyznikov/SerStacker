/*
 * QMeasureMinMax.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureMinMax.h"

template<class T>
static int compute_min_(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value)
{
  const int rows =
      image.rows;

  const int cols =
      image.cols;

  const int channels =
      image.channels();

  const cv::Mat_<T> src =
      image;

  const cv::Mat1b msk =
      mask;

  *value =
      cv::Scalar::all(HUGE_VAL);

  if( msk.empty() ) {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];

      for( int x = 0; x < cols; ++x ) {

        for( int c = 0; c < channels; ++c ) {

          const T &v = sp[x * channels + c];

          if( v < (*value)(c) ) {
            (*value)(c) = v;
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = 0; x < cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < channels; ++c ) {

            const T &v = sp[x * channels + c];

            if( v < (*value)(c) ) {
              (*value)(c) = v;
            }
          }
        }
      }
    }
  }

  return channels;
}

template<class T>
static int compute_min_non_zero_(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value)
{
  const int rows =
      image.rows;

  const int cols =
      image.cols;

  const int channels =
      image.channels();

  const cv::Mat_<T> src =
      image;

  const cv::Mat1b msk =
      mask;

  *value =
      cv::Scalar::all(HUGE_VAL);

  if( msk.empty() ) {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];

      for( int x = 0; x < cols; ++x ) {

        for( int c = 0; c < channels; ++c ) {

          const T & v = sp[x * channels + c];

          if( v && v < (*value)(c) ) {
            (*value)(c) = v;
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = 0; x < cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < channels; ++c ) {

            const T & v = sp[x * channels + c];

            if( v && v < (*value)(c) ) {
              (*value)(c) = v;
            }
          }
        }
      }
    }
  }

  return channels;
}



template<class T>
static int compute_max_(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value)
{
  const int rows =
      image.rows;

  const int cols =
      image.cols;

  const int channels =
      image.channels();

  const cv::Mat_<T> src =
      image;

  const cv::Mat1b msk =
      mask;

  *value =
      cv::Scalar::all(-HUGE_VAL);

  if( msk.empty() ) {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];

      for( int x = 0; x < cols; ++x ) {

        for( int c = 0; c < channels; ++c ) {

          const T &v = sp[x * channels + c];

          if( v > (*value)(c) ) {
            (*value)(c) = v;
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < rows; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = 0; x < cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < channels; ++c ) {

            const T &v = sp[x * channels + c];

            if( v > (*value)(c) ) {
              (*value)(c) = v;
            }

          }
        }
      }
    }
  }

  return channels;
}

QMeasureMinValue::QMeasureMinValue() :
    Base("Min", "Get minimal value from image ROI")
{
}

int QMeasureMinValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value) const
{
  switch (image.depth()) {
    case CV_8U:
      return compute_min_<uint8_t>(image, mask, value);
    case CV_8S:
      return compute_min_<int8_t>(image, mask, value);
    case CV_16U:
      return compute_min_<uint16_t>(image, mask, value);
    case CV_16S:
      return compute_min_<int16_t>(image, mask, value);
    case CV_32S:
      return compute_min_<int32_t>(image, mask, value);
    case CV_32F:
      return compute_min_<float>(image, mask, value);
    case CV_64F:
      return compute_min_<double>(image, mask, value);
  }

  return 0;
}

QMeasureMaxValue::QMeasureMaxValue() :
    Base("Max", "Get maximal value from image ROI")
{
}


int QMeasureMaxValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value) const
{
  switch (image.depth()) {
    case CV_8U:
      return compute_max_<uint8_t>(image, mask, value);
    case CV_8S:
      return compute_max_<int8_t>(image, mask, value);
    case CV_16U:
      return compute_max_<uint16_t>(image, mask, value);
    case CV_16S:
      return compute_max_<int16_t>(image, mask, value);
    case CV_32S:
      return compute_max_<int32_t>(image, mask, value);
    case CV_32F:
      return compute_max_<float>(image, mask, value);
    case CV_64F:
      return compute_max_<double>(image, mask, value);
  }

  return 0;
}

QMeasureMinNonZeroValue::QMeasureMinNonZeroValue() :
    Base("MinNonZero", "Get maximal non-zero value from image ROI")
{
}

int QMeasureMinNonZeroValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  switch (image.depth()) {
    case CV_8U:
      return compute_min_non_zero_<uint8_t>(image, mask, output_value);
    case CV_8S:
      return compute_min_non_zero_<int8_t>(image, mask, output_value);
    case CV_16U:
      return compute_min_non_zero_<uint16_t>(image, mask, output_value);
    case CV_16S:
      return compute_min_non_zero_<int16_t>(image, mask, output_value);
    case CV_32S:
      return compute_min_non_zero_<int32_t>(image, mask, output_value);
    case CV_32F:
      return compute_min_non_zero_<float>(image, mask, output_value);
    case CV_64F:
      return compute_min_non_zero_<double>(image, mask, output_value);
  }

  return 0;
}

