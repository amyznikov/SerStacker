/*
 * QMeasureLC.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureLC.h"

QMeasureLC::QMeasureLC() :
    Base("LC", "LC")
{
}

int QMeasureLC::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
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

  *value = c_local_contrast_measure::compute(src(rc));

  return avgchannel_ ? 1 : src.channels();
}

bool QMeasureLC::hasOptions() const
{
  return true;
}

QMeasureSettingsWidget* QMeasureLC::createSettingsWidget(QWidget * parent) const
{
  return new QLCMeasureSettingsWidget(parent);
}

QLCMeasureSettingsWidget::QLCMeasureSettingsWidget(QWidget * parent) :
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

  dscale_ctl =
      add_numeric_box<int>("dscale:",
          "",
          [this](int v) {
            if ( measure_ && v != measure_->dscale() ) {
              measure_->set_dscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( measure_ ) {
              *v = measure_->dscale();
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("eps:",
          "",
          [this](double v) {
            if ( measure_ && v != measure_->eps() ) {
              measure_->set_eps(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( measure_ ) {
              *v = measure_->eps();
              return true;
            }
            return false;
          });

  updateControls();
}
