/*
 * QLocalContrastMeasureOptions.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "QLocalContrastMeasureOptions.h"

QLocalContrastMeasureOptions::QLocalContrastMeasureOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
}

QLocalContrastMeasureOptions::QLocalContrastMeasureOptions(QWidget * parent) :
    ThisClass("QLocalContrastMeasureOptions", parent)
{
  avgchannel_ctl =
      add_checkbox("Average color channels",
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

void QLocalContrastMeasureOptions::set_measure_options(c_local_contrast_measure * options)
{
  options_ = options;
  updateControls();
}

const c_local_contrast_measure * QLocalContrastMeasureOptions::measure_options() const
{
  return options_;
}

void QLocalContrastMeasureOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( !options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void QLocalContrastMeasureOptions::onload(QSettings & settings)
{
  Base::onload(settings);
}
