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

int QMeasureNormalizedVariance::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_normalized_variance_measure::compute(image);
  return _opts.avgchannel ? 1 : image.channels();
}


QNormalizedVarianceSettingsWidget::QNormalizedVarianceSettingsWidget(QWidget * parent) :
    Base(parent)
{
  updateControls();
}
