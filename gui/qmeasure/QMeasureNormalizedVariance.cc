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

int QMeasureNormalizedVariance::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_normalized_variance_measure::compute(image);
  return avgchannel_ ? 1 : image.channels();
}

bool QMeasureNormalizedVariance::hasOptions() const
{
  return true;
}

QMeasureSettingsWidget* QMeasureNormalizedVariance::createSettingsWidget(QWidget * parent) const
{
  return new QNormalizedVarianceSettingsWidget(parent);
}

QNormalizedVarianceSettingsWidget::QNormalizedVarianceSettingsWidget(QWidget * parent) :
    Base(parent)
{
  avgc_ctl =
      add_checkbox("Average color channels:",
          "",
          [this](bool checked) {
            if ( measure_ && measure_->avgchannel() != checked ) {
              measure_->set_avgchannel(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( measure_ ) {
              * checked = measure_->avgchannel();
              return true;
            }
            return false;
          });

  updateControls();
}
