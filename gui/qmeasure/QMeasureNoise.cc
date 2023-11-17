/*
 * QMeasureNoise.cc
 *
 *  Created on: Apr 16, 2023
 *      Author: amyznikov
 */

#include "QMeasureNoise.h"
#include <core/proc/estimate_noise.h>

QMeasureNoise::QMeasureNoise() :
  Base("Noise", "Measure image noise stdev on image ROI")
{
}

QMeasureSettingsWidget * QMeasureNoise::createSettingsWidget(QWidget * parent) const
{
  return new QNoiseMeasureSettingsWidget(parent);
}

int QMeasureNoise::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  * output_value = estimate_noise(image, cv::noArray(), mask);
  return image.channels();
}

