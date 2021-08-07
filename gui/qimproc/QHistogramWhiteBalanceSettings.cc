/*
 * QHistogramWhiteBalanceSettings.cc
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#include "QHistogramWhiteBalanceSettings.h"

const QHistogramWhiteBalanceSettings::ClassFactory QHistogramWhiteBalanceSettings::classFactory;

QHistogramWhiteBalanceSettings::QHistogramWhiteBalanceSettings(const c_histogram_white_balance_routine::ptr & routine, QWidget * parent) :
    Base(&classFactory, routine, parent)
{
  lclip_ctl = add_numeric_box("lclip [%]", &routine_,
      &c_histogram_white_balance_routine::lclip,
      &c_histogram_white_balance_routine::set_lclip);

  hclip_ctl = add_numeric_box("hclip [%]", &routine_,
      &c_histogram_white_balance_routine::hclip,
      &c_histogram_white_balance_routine::set_hclip);

  enable_threshold_ctl = add_checkbox("threshold", &routine_,
      &c_histogram_white_balance_routine::enable_threshold,
      &c_histogram_white_balance_routine::set_enable_threshold);

  threshold_ctl = add_numeric_box("threshold value", &routine_,
      &c_histogram_white_balance_routine::threshold,
      &c_histogram_white_balance_routine::set_threshold);

  updateControls();
}

void QHistogramWhiteBalanceSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {

    lclip_ctl->setValue(routine_->lclip());
    hclip_ctl->setValue(routine_->hclip());
    enable_threshold_ctl->setChecked(routine_->enable_threshold());
    threshold_ctl->setValue(routine_->threshold());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}

