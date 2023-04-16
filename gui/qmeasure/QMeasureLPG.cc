/*
 * QLPGMeasure.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */
#include "QMeasureLPG.h"

QMeasureLPG::QMeasureLPG() :
    Base("LPG", "Compute LPG value over image ROI")
{
}

int QMeasureLPG::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_lpg_sharpness_measure::compute(image);
  return avgchannel_ ? 1 : image.channels();
}

bool QMeasureLPG::hasOptions() const
{
  return true;
}

QMeasureSettingsWidget* QMeasureLPG::createSettingsWidget(QWidget * parent) const
{
  return new QLPGMeasureSettingsWidget(parent);
}

QLPGMeasureSettingsWidget::QLPGMeasureSettingsWidget(QWidget * parent) :
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

  k_ctl =
      add_numeric_box<double>("k:",
          "",
          [this](double v) {
            if ( measure_ && v != measure_->k() ) {
              measure_->set_k(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( measure_ ) {
              *v = measure_->k();
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

  uscale_ctl =
      add_numeric_box<int>("uscale:",
          "",
          [this](int v) {
            if ( measure_ && v != measure_->uscale() ) {
              measure_->set_uscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( measure_ ) {
              *v = measure_->uscale();
              return true;
            }
            return false;
          });

  square_ctl =
      add_checkbox("squared:",
          "",
          [this](bool checked) {
            if ( measure_ && measure_->squared() != checked ) {
              measure_->set_squared(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( measure_ ) {
              * checked = measure_->squared();
              return true;
            }
            return false;
          });

  updateControls();
}
