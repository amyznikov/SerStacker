/*
 * QMeasureMinMax.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureMinMax.h"

template<class T>
static int compute_min_(cv::InputArray image, cv::InputArray mask, const cv::Rect & rc, cv::Scalar * value)
{
  const cv::Mat_<T> src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  *value =
      cv::Scalar::all(HUGE_VAL);

  if( msk.empty() ) {

    for( int y = rc.y, ymax = rc.y + rc.height; y < ymax; ++y ) {

      const T *sp = src[y];

      for( int x = rc.x, xmax = rc.x + rc.width; x < xmax; ++x ) {

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

    for( int y = rc.y, ymax = rc.y + rc.height; y < ymax; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = rc.x, xmax = rc.x + rc.width; x < xmax; ++x ) {
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
static int compute_max_(cv::InputArray image, cv::InputArray mask, const cv::Rect & rc, cv::Scalar * value)
{
  const cv::Mat_<T> src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  *value =
      cv::Scalar::all(-HUGE_VAL);

  if( msk.empty() ) {

    for( int y = rc.y, ymax = rc.y + rc.height; y < ymax; ++y ) {

      const T *sp = src[y];

      for( int x = rc.x, xmax = rc.x + rc.width; x < xmax; ++x ) {

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

    for( int y = rc.y, ymax = rc.y + rc.height; y < ymax; ++y ) {

      const T *sp = src[y];
      const uint8_t *mp = msk[y];

      for( int x = rc.x, xmax = rc.x + rc.width; x < xmax; ++x ) {
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

int QMeasureMinValue::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
{
  cv::Rect rc;

  if( !adjust_roi(roi, image.size(), &rc) ) {
    return 0;
  }

  switch (image.depth()) {
    case CV_8U:
      return compute_min_<uint8_t>(image, mask, rc, value);
    case CV_8S:
      return compute_min_<int8_t>(image, mask, rc, value);
    case CV_16U:
      return compute_min_<uint16_t>(image, mask, rc, value);
    case CV_16S:
      return compute_min_<int16_t>(image, mask, rc, value);
    case CV_32S:
      return compute_min_<int32_t>(image, mask, rc, value);
    case CV_32F:
      return compute_min_<float>(image, mask, rc, value);
    case CV_64F:
      return compute_min_<double>(image, mask, rc, value);
  }

  return 0;
}

QMeasureMaxValue::QMeasureMaxValue() :
    Base("Max", "Get maximal value from image ROI")
{
}

int QMeasureMaxValue::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
{
  cv::Rect rc;

  if( !adjust_roi(roi, image.size(), &rc) ) {
    return 0;
  }

  switch (image.depth()) {
    case CV_8U:
      return compute_max_<uint8_t>(image, mask, rc, value);
    case CV_8S:
      return compute_max_<int8_t>(image, mask, rc, value);
    case CV_16U:
      return compute_max_<uint16_t>(image, mask, rc, value);
    case CV_16S:
      return compute_max_<int16_t>(image, mask, rc, value);
    case CV_32S:
      return compute_max_<int32_t>(image, mask, rc, value);
    case CV_32F:
      return compute_max_<float>(image, mask, rc, value);
    case CV_64F:
      return compute_max_<double>(image, mask, rc, value);
  }

  return 0;
}
