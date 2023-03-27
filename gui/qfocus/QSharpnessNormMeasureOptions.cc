/*
 * QSharpnessNormMeasureOptions.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "QSharpnessNormMeasureOptions.h"

QSharpnessNormMeasureOptions::QSharpnessNormMeasureOptions(QWidget * parent) :
    ThisClass("QSharpnessNormMeasureOptions", parent)
{

}

QSharpnessNormMeasureOptions::QSharpnessNormMeasureOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{

  norm_type_ctl =
      add_enum_combobox<NormType>("Norm type:",
          "",
          [this](NormType v) {
            if ( options_ && options_->norm_type() != v ) {
              options_->set_norm_type(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](NormType * v) {
            if ( options_ ) {
              *v = options_->norm_type();
              return true;
            }
            return false;
          });

  sigma_ctl =
      add_numeric_box<double>("Low-pass sigma [pix]:",
          "",
          [this](double v) {
            if ( options_ && v > 0 && options_->sigma() != v ) {
              options_->set_sigma(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              * v= options_->sigma();
              return true;
            }
            return false;
          });
}

void QSharpnessNormMeasureOptions::set_measure_options(c_sharpness_norm_measure * options)
{
  options_ = options;
  updateControls();
}

const c_sharpness_norm_measure * QSharpnessNormMeasureOptions::measure_options() const
{
  return options_;
}

void QSharpnessNormMeasureOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( !options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void QSharpnessNormMeasureOptions::onload(QSettings & settings)
{
  Base::onload(settings);
}
