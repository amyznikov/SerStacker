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

int QMeasureLPG::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_lpg_sharpness_measure::compute(image, mask);
  return _opts.avgchannel ? 1 : image.channels();
}


QLPGMeasureSettingsWidget::QLPGMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  k_ctl =
      add_numeric_box<double>("k:",
          "",
          [this](double v) {
            if ( _measure && v != _measure->k() ) {
              _measure->set_k(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _measure ) {
              *v = _measure->k();
              return true;
            }
            return false;
          });

  p_ctl =
      add_numeric_box<double>("p:",
          "power",
          [this](double v) {
            if ( _measure && _measure->p() != v ) {
              _measure->set_p(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * checked) {
            if ( _measure ) {
              * checked = _measure->p();
              return true;
            }
            return false;
          });

  dscale_ctl =
      add_numeric_box<int>("dscale:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->dscale() ) {
              _measure->set_dscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->dscale();
              return true;
            }
            return false;
          });

  uscale_ctl =
      add_numeric_box<int>("uscale:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->uscale() ) {
              _measure->set_uscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->uscale();
              return true;
            }
            return false;
          });


  updateControls();
}
