/*
 * QMeasure.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */
#include "QMeasure.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

int QMeasure::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * output_value) const
{
  const cv::Mat src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  cv::Rect rc;
  cv::Mat compute_mask;

  if( !adjust_roi(roi, src.size(), &rc) ) {
    return 0;
  }

  if ( !skip_zero_pixels_ ) {
    if ( !msk.empty() ) {
      compute_mask = msk(rc);
    }
  }
  else {
    cv::compare(src(rc), 0, compute_mask, cv::CMP_NE);
    if ( !msk.empty() ) {
      compute_mask.setTo(cv::Scalar::all(0), ~msk(rc));
    }
  }

  return compute_measure(src(rc), compute_mask, output_value);
}

bool QMeasure::adjust_roi(const cv::Rect & src_roi, const cv::Size & image_size, cv::Rect * dst_roi)
{
  const int l =
      (std::min)(image_size.width - 1, (std::max)(0, src_roi.x));

  const int t =
      (std::min)(image_size.height - 1, (std::max)(0, src_roi.y));

  const int r =
      (std::min)(image_size.width - 1, (std::max)(0, src_roi.x + src_roi.width - 1));

  const int b =
      (std::min)(image_size.height - 1, (std::max)(0, src_roi.y + src_roi.height - 1));

  *dst_roi = cv::Rect(l, t, r - l + 1, b - t + 1);

  return !dst_roi->empty();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QMeasureCentralPixelValue::QMeasureCentralPixelValue() :
  Base("Value", "Get pixel value from center point of image ROI")
{
}

QMeasureSettingsWidget * QMeasureCentralPixelValue::createSettingsWidget(QWidget * parent) const
{
  return new QMeasureCentralPixelValueSettingsWidget(parent) ;
}


int QMeasureCentralPixelValue::compute_measure(const cv::Mat & image, const cv::Mat & mask,
    cv::Scalar * output_value) const
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
