/*
 * QMeasureNormalizedVariance.cc
 *
 *  Created on: Apr 11, 2023
 *      Author: amyznikov
 */

#include "QMeasureNormalizedVariance.h"

QMeasureNormalizedVariance::QMeasureNormalizedVariance() :
  Base("NormalizedVariance", "Normalized Variance over image ROI")
{
}

QMeasureSettingsWidget* QMeasureNormalizedVariance::createSettingsWidget(QWidget * parent) const
{
  return new QNormalizedVarianceSettingsWidget(parent);
}

void QMeasureNormalizedVariance::setAverageColorChannels(bool v)
{
  c_normalized_variance_measure::set_avgchannel(average_color_channels_ = v);
}

bool QMeasureNormalizedVariance::averageColorChannels() const
{
  return c_normalized_variance_measure::avgchannel();
}

int QMeasureNormalizedVariance::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_normalized_variance_measure::compute(image);
  return avgchannel_ ? 1 : image.channels();
}


QNormalizedVarianceSettingsWidget::QNormalizedVarianceSettingsWidget(QWidget * parent) :
    Base(parent)
{
  updateControls();
}
