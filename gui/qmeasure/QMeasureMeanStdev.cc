/*
 * QMeasureMeanStdev.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureMeanStdev.h"

QMeasureMeanValue::QMeasureMeanValue() :
  Base("Mean", "Compute mean value over image ROI")
{
}

int QMeasureMeanValue::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const
{
  const cv::Mat src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  cv::Rect rc;

  if( !adjust_roi(roi, src.size(), &rc) ) {
    return 0;
  }

  *value =
      cv::mean(src(rc), msk.empty() ?
          cv::noArray() :
          msk(rc));

  return cn;
}

QMeasureStdevValue::QMeasureStdevValue() :
    Base("Stdev", "Compute stdev value over image ROI")
{
}

int QMeasureStdevValue::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const
{
  const cv::Mat src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  cv::Rect rc;

  if( !adjust_roi(roi, src.size(), &rc) ) {
    return 0;
  }

  cv::Scalar m;

  cv::meanStdDev(src(rc), m, *value, msk.empty() ?
      cv::noArray() :
      msk(rc));

  return cn;
}
