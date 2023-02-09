/*
 * QNormalizedVarianceMeasureOptions.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "QNormalizedVarianceMeasureOptions.h"

QNormalizedVarianceMeasureOptions::QNormalizedVarianceMeasureOptions(QWidget * parent) :
    ThisClass("QNormalizedVarianceMeasureOptions", parent)
{
}

QNormalizedVarianceMeasureOptions::QNormalizedVarianceMeasureOptions(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
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

void QNormalizedVarianceMeasureOptions::set_measure_options(c_normalized_variance_measure * options)
{
  options_ = options;
  updateControls();
}

const c_normalized_variance_measure * QNormalizedVarianceMeasureOptions::measure_options() const
{
  return options_;
}

void QNormalizedVarianceMeasureOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( !options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void QNormalizedVarianceMeasureOptions::onload(QSettings & settings)
{
  Base::onload(settings);
}
