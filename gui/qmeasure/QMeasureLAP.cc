/*
 * QMeasureLAP.cc
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#include "QMeasureLAP.h"
#include<core/proc/sharpness_measure/c_laplacian_sharpness_measure.h>

QMeasureLAP::QMeasureLAP() :
  Base("LAP", "LAP")
{
}

QMeasureSettingsWidget* QMeasureLAP::createSettingsWidget(QWidget * parent) const
{
  return new QMeasureLAPSettingsWidget(parent);
}

void QMeasureLAP::set_dscale(int v)
{
  dscale_ = v;
}

int QMeasureLAP::dscale() const
{
  return dscale_;
}

void QMeasureLAP::set_se_size(const cv::Size & v)
{
  se_size_ = v;
}

const cv::Size & QMeasureLAP::se_size() const
{
  return se_size_;
}

int QMeasureLAP::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  c_laplacian_sharpness_measure::compute(image, mask, dscale_, se_size_, output_value);
  return 1;
}


QMeasureLAPSettingsWidget::QMeasureLAPSettingsWidget(QWidget * parent) :
    Base(parent)
{
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

  se_size_ctl =
      add_numeric_box<cv::Size>("SE size:",
          "",
          [this](const cv::Size & v) {
            if ( measure_ && v != measure_->se_size() ) {
              measure_->set_se_size(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * v) {
            if ( measure_ ) {
              *v = measure_->se_size();
              return true;
            }
            return false;
          });

  updateControls();
}
