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

QMeasureSettingsWidget* QMeasureLPG::createSettingsWidget(QWidget * parent) const
{
  return new QLPGMeasureSettingsWidget(parent);
}

void QMeasureLPG::setAverageColorChannels(bool v)
{
  options_.avgchannel = v;
  average_color_channels_ = options_.avgchannel;
}

bool QMeasureLPG::averageColorChannels() const
{
  return options_.avgchannel;
}

int QMeasureLPG::compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_lpg_sharpness_measure::compute(image, mask);
  return options_.avgchannel ? 1 : image.channels();
}


QLPGMeasureSettingsWidget::QLPGMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
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

  p_ctl =
      add_numeric_box<double>("p:",
          "power",
          [this](double v) {
            if ( measure_ && measure_->p() != v ) {
              measure_->set_p(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * checked) {
            if ( measure_ ) {
              * checked = measure_->p();
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


  updateControls();
}
