/*
 * QMeasureLocalVariance.cc
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#include "QMeasureLocalVariance.h"

QMeasureLocalVariance::QMeasureLocalVariance() :
    Base("LocalVariance", "Compute LocalVariance MAP average over image ROI")
{
}

QMeasureSettingsWidget* QMeasureLocalVariance::createSettingsWidget(QWidget * parent) const
{
  return new QLocalVarianceMeasureSettingsWidget(parent);
}

int QMeasureLocalVariance::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_local_variance_sharpness_measure::compute(image, mask);
  return 1;
}

// TODO: implement via setupControls()
// #include <gui/widgets/QSettingsWidgetTemplate.h>
QLocalVarianceMeasureSettingsWidget::QLocalVarianceMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  p_ctl =
      add_numeric_box<double>("p:",
          "power",
          [this](double v) {
            if ( _measure && _measure->opts().p != v ) {
              _measure->opts().p = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * checked) {
            if ( _measure ) {
              * checked = _measure->opts().p;
              return true;
            }
            return false;
          });

  kradius_ctl =
      add_numeric_box<int>("kradius:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->opts().kradius ) {
              _measure->opts().kradius = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->opts().kradius;
              return true;
            }
            return false;
          });


  dscale_ctl =
      add_numeric_box<int>("dscale:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->opts().dscale ) {
              _measure->opts().dscale = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->opts().dscale;
              return true;
            }
            return false;
          });

  uscale_ctl =
      add_numeric_box<int>("uscale:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->opts().uscale ) {
              _measure->opts().uscale = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->opts().uscale;
              return true;
            }
            return false;
          });
}
