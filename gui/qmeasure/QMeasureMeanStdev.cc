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

QMeasureSettingsWidget* QMeasureMeanValue::createSettingsWidget(QWidget * parent) const
{
  return new QMeanValueMeasureSettingsWidget(parent);
}

int QMeasureMeanValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  if ( mask.empty() || mask.channels() == 1 ) {
    *output_value =
        cv::mean(image, mask);
  }
  else if ( mask.channels() == image.channels() ) {

    std::vector<cv::Mat > image_channels;
    std::vector<cv::Mat > mask_channels;

    cv::split(image, image_channels);
    cv::split(mask, mask_channels);

    for( int c = 0; c < image_channels.size(); ++c ) {
      (*output_value)[c] =
          cv::mean(image_channels[c], mask_channels[c])[0];
    }
  }
  else {
    return 0;
  }

  return image.channels();
}

QMeasureStdevValue::QMeasureStdevValue() :
    Base("Stdev", "Compute stdev value over image ROI")
{
}

QMeasureSettingsWidget* QMeasureStdevValue::createSettingsWidget(QWidget * parent) const
{
  return new QStdevValueMeasureSettingsWidget(parent);
}

int QMeasureStdevValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  if ( mask.empty() || mask.channels() == 1 ) {
    cv::Scalar m;
    cv::meanStdDev(image, m, *output_value, mask);
  }
  else if ( mask.channels() == image.channels() ) {

    std::vector<cv::Mat > image_channels;
    std::vector<cv::Mat > mask_channels;
    cv::Scalar m, s;

    cv::split(image, image_channels);
    cv::split(mask, mask_channels);

    for( int c = 0; c < image_channels.size(); ++c ) {

      cv::meanStdDev(image_channels[c], m, s, mask_channels[c]);

      (*output_value)[c] = s[0];
    }
  }
  else {
    return 0;
  }

  return image.channels();
}
