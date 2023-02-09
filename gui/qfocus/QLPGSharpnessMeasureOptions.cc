/*
 * QLPGSharpnessMeasureOptions.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "QLPGSharpnessMeasureOptions.h"

QLPGSharpnessMeasureOptions::QLPGSharpnessMeasureOptions(QWidget * parent) :
    ThisClass("QLPGSharpnessMeasureOptions", parent)
{
}

QLPGSharpnessMeasureOptions::QLPGSharpnessMeasureOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  avgc_ctl =
      add_checkbox("Average color channels:",
          [this](bool checked) {
            if ( options_ && options_->avgchannel() != checked ) {
              options_->set_avgchannel(checked);
              Q_EMIT parameterChanged();
            }
          });

  k_ctl =
      add_numeric_box<double>("k:",
          [this](double v) {
            if ( options_ && v != options_->k() ) {
              options_->set_k(v);
              Q_EMIT parameterChanged();
            }
          });

  dscale_ctl =
      add_numeric_box<double>("dscale:",
          [this](double v) {
            if ( options_ && v != options_->dscale() ) {
              options_->set_dscale(v);
              Q_EMIT parameterChanged();
            }
          });

  uscale_ctl =
      add_numeric_box<double>("uscale:",
          [this](double v) {
            if ( options_ && v != options_->uscale() ) {
              options_->set_uscale(v);
              Q_EMIT parameterChanged();
            }
          });

  square_ctl =
      add_checkbox("squared:",
          [this](bool checked) {
            if ( options_ && options_->squared() != checked ) {
              options_->set_squared(checked);
              Q_EMIT parameterChanged();
            }
          });


  updateControls();
}

void QLPGSharpnessMeasureOptions::set_measure_options(c_lpg_sharpness_measure * options)
{
  options_ = options;
  updateControls();
}

const c_lpg_sharpness_measure* QLPGSharpnessMeasureOptions::measure_options() const
{
  return options_;
}

void QLPGSharpnessMeasureOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    k_ctl->setValue(options_->k());
    dscale_ctl->setValue(options_->dscale());
    uscale_ctl->setValue(options_->uscale());
    square_ctl->setChecked(options_->squared());
    avgc_ctl->setChecked(options_->avgchannel());

    setEnabled(true);
  }
}

void QLPGSharpnessMeasureOptions::onload(QSettings & settings)
{

}
