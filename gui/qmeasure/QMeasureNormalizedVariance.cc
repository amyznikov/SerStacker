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


int QMeasureNormalizedVariance::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
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

  *value = c_normalized_variance_measure::compute(src(rc));

  return avgchannel_ ? 1 : src.channels();
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
