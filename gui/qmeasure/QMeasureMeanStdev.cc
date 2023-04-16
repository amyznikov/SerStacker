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

int QMeasureMeanValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value =
      cv::mean(image, mask);
  return image.channels();
}

QMeasureStdevValue::QMeasureStdevValue() :
    Base("Stdev", "Compute stdev value over image ROI")
{
}

int QMeasureStdevValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  cv::Scalar m;
  cv::meanStdDev(image, m, *output_value, mask);
  return image.channels();
}
