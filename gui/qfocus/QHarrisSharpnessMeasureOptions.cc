/*
 * QHarrisSharpnessMeasureOptions.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "QHarrisSharpnessMeasureOptions.h"

QHarrisSharpnessMeasureOptions::QHarrisSharpnessMeasureOptions(QWidget * parent) :
    ThisClass("QHarrisSharpnessMeasureOptions", parent)
{
}

QHarrisSharpnessMeasureOptions::QHarrisSharpnessMeasureOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  avgchannel_ctl =
      add_checkbox("Average color channels",
          "",
          [this](bool checked) {
            if ( options_ && options_->avgchannel() != checked ) {
              options_->set_avgchannel(checked);
              Q_EMIT parameterChanged();

            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->avgchannel();
              return true;
            }
            return false;
          });
}


void QHarrisSharpnessMeasureOptions::set_measure_options(c_harris_sharpness_measure * options)
{
  options_ = options;
  updateControls();
}

const c_harris_sharpness_measure * QHarrisSharpnessMeasureOptions::measure_options() const
{
  return options_;
}

void QHarrisSharpnessMeasureOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( !options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void QHarrisSharpnessMeasureOptions::onload(QSettings & settings)
{
  Base::onload(settings);
}
