/*
 * QMeasure.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */
#include "QMeasure.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

QMeasureCentralPixelValue::QMeasureCentralPixelValue() :
  Base("Value", "Get pixel value from center point of image ROI")
{
}

QMeasureSettingsWidget * QMeasureCentralPixelValue::createSettingsWidget(QWidget * parent) const
{
  return new QMeasureCentralPixelValueSettingsWidget(parent) ;
}


int QMeasureCentralPixelValue::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  const int ix = image.cols / 2;
  const int iy = image.rows / 2;
  const int cn = image.channels();

  switch (image.depth()) {
    case CV_8U: {
      const uint8_t *imgp =
          image.ptr<const uint8_t>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_8S: {
      const int8_t *imgp =
          image.ptr<const int8_t>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_16U: {
      const uint16_t *imgp =
          image.ptr<const uint16_t>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_16S: {
      const int16_t *imgp =
          image.ptr<const int16_t>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_32S: {
      const int32_t *imgp =
          image.ptr<const int32_t>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_32F: {
      const float *imgp =
          image.ptr<const float>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    case CV_64F: {
      const double *imgp =
          image.ptr<const double>(iy);
      for( int c = 0; c < cn; ++c ) {
        output_value->val[c] = imgp[ix * cn + c];
      }
      break;
    }
    default:
      return 0;
  }

  return cn;
}
