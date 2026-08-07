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

QLocalVarianceMeasureSettingsWidget::QLocalVarianceMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  color_channel_ctl =
      add_enum_combobox<color_channel_type>("channel:",
          "Select reference channel for computation",
          [this](color_channel_type v) {
            if ( _measure && v != _measure->opts.channel ) {
              _measure->opts.channel = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](color_channel_type * v) {
            if ( _measure ) {
              *v = _measure->opts.channel;
              return true;
            }
            return false;
          });


  dscale_ctl =
      add_numeric_box<int>("dscale:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->opts.dscale ) {
              _measure->opts.dscale = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->opts.dscale;
              return true;
            }
            return false;
          });

  kradius_ctl =
      add_numeric_box<int>("kradius:",
          "",
          [this](int v) {
            if ( _measure && v != _measure->opts.kradius ) {
              _measure->opts.kradius = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _measure ) {
              *v = _measure->opts.kradius;
              return true;
            }
            return false;
          });

//  uscale_ctl =
//      add_numeric_box<int>("uscale:",
//          "",
//          [this](int v) {
//            if ( _measure && v != _measure->opts.uscale ) {
//              _measure->opts.uscale = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * v) {
//            if ( _measure ) {
//              *v = _measure->opts.uscale;
//              return true;
//            }
//            return false;
//          });
}
