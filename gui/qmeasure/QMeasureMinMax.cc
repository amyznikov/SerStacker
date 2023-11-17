/*
 * QMeasureMinMax.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureMinMax.h"
#include <core/proc/reduce_channels.h>


QMeasureMinValue::QMeasureMinValue() :
    Base("Min", "Get minimal value from image ROI")
{
}

QMeasureSettingsWidget * QMeasureMinValue::createSettingsWidget(QWidget * parent) const
{
  return new QMinValueMeasureSettingsWidget(parent);
}

int QMeasureMinValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  double min, max;

  if( average_color_channels_ && image.channels() > 1 ) {

    if( mask.empty() || mask.channels() == 1 ) {
      cv::minMaxLoc(image, &min, &max,
          nullptr, nullptr,
          mask);
    }
    else {

      cv::Mat compute_mask;

      reduce_color_channels(mask, compute_mask,
          cv::REDUCE_MAX);

      cv::minMaxLoc(image, &min, &max,
          nullptr, nullptr,
          compute_mask);
    }

    (*output_value)[0] = min;
    return 1;
  }


  if ( mask.empty() || mask.channels() == 1 ) {

    cv::minMaxLoc(image, &min, &max,
        nullptr, nullptr,
        mask);

    (*output_value)[0] = min;
  }
  else if ( mask.channels() == image.channels() ) {

    std::vector<cv::Mat > image_channels;
    std::vector<cv::Mat > mask_channels;

    cv::split(image, image_channels);
    cv::split(mask, mask_channels);

    for( int c = 0; c < image_channels.size(); ++c ) {

      cv::minMaxLoc(image_channels[c], &min, &max,
          nullptr, nullptr,
          mask_channels[c]);

      (*output_value)[c] = min;
    }
  }

  else {
    // bad input

    return 0;
  }

  return image.channels();
}

QMeasureMaxValue::QMeasureMaxValue() :
    Base("Max", "Get maximal value from image ROI")
{
}

QMeasureSettingsWidget * QMeasureMaxValue::createSettingsWidget(QWidget * parent) const
{
  return new QMaxValueMeasureSettingsWidget(parent);
}


int QMeasureMaxValue::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  double min, max;

  if( average_color_channels_ && image.channels() > 1 ) {

    if( mask.empty() || mask.channels() == 1 ) {

      cv::minMaxLoc(image, &min, &max,
          nullptr, nullptr,
          mask);
    }
    else {

      cv::Mat compute_mask;

      reduce_color_channels(mask, compute_mask,
          cv::REDUCE_MAX);

      cv::minMaxLoc(image, &min, &max,
          nullptr, nullptr,
          compute_mask);
    }

    (*output_value)[0] = max;
    return 1;
  }


  if ( mask.empty() || mask.channels() == 1 ) {

    cv::minMaxLoc(image, &min, &max,
        nullptr, nullptr,
        mask);

    (*output_value)[0] = max;
  }
  else if ( mask.channels() == image.channels() ) {

    std::vector<cv::Mat > image_channels;
    std::vector<cv::Mat > mask_channels;

    cv::split(image, image_channels);
    cv::split(mask, mask_channels);

    for( int c = 0; c < image_channels.size(); ++c ) {

      cv::minMaxLoc(image_channels[c], &min, &max,
          nullptr, nullptr,
          mask_channels[c]);

      (*output_value)[c] = max;
    }
  }

  else {
    // bad input

    return 0;
  }

  return image.channels();
}

