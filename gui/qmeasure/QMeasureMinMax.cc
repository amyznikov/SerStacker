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
  const cv::Mat_<T> src =
      image;

  const cv::Mat1b msk =
      mask;

  const int cn =
      src.channels();

  *value =
      cv::Scalar::all(HUGE_VAL);

  if( msk.empty() ) {

    for( int y = 0; y < image.rows; ++y ) {

      const T *sp = src[y];

      for( int x = 0; x < image.cols; ++x ) {

        for( int c = 0; c < cn; ++c ) {

          const T &v = sp[x * cn + c];

          if( v < (*value)(c) ) {
            (*value)(c) = v;
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < image.rows; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = 0; x < image.cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < cn; ++c ) {

            const T &v = sp[x * cn + c];

            if( v < (*value)(c) ) {
              (*value)(c) = v;
            }
          }
        }
      }
    }
  }

  return cn;
}


template<class T>
static int compute_max_(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * value)
{
  const cv::Mat_<T> src =
      image;

  const cv::Mat1b msk =
      mask;

  const int cn =
      src.channels();

  *value =
      cv::Scalar::all(-HUGE_VAL);

  if( msk.empty() ) {

    for( int y = 0; y < image.rows; ++y ) {

      const T *sp = src[y];

      for( int x = 0; x < image.cols; ++x ) {

        for( int c = 0; c < cn; ++c ) {

          const T &v = sp[x * cn + c];

          if( v > (*value)(c) ) {
            (*value)(c) = v;
          }
        }
      }
    }
  }
  else {

    for( int y = 0; y < image.rows; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = 0; x < image.cols; ++x ) {
        if( mp[x] ) {
          for( int c = 0; c < cn; ++c ) {

            const T &v = sp[x * cn + c];

            if( v > (*value)(c) ) {
              (*value)(c) = v;
            }

          }
        }
      }
    }
  }

  return cn;
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

