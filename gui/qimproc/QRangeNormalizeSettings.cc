/*
 * QRangeNormalizeSettings.cc
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#include "QRangeNormalizeSettings.h"


const QRangeNormalizeSettings::ClassFactory QRangeNormalizeSettings::classFactory;

QRangeNormalizeSettings::QRangeNormalizeSettings(const c_range_normalize_routine::ptr & routine, QWidget * parent) :
    Base(&classFactory, routine, parent)
{
  auto_input_range_ctl = add_checkbox("Auto input range",
      &c_range_normalize_routine::auto_input_range,
      &c_range_normalize_routine::set_auto_input_range);

  input_min_ctl = add_numeric_box("input min",
      &c_range_normalize_routine::input_min,
      &c_range_normalize_routine::set_input_min);

  input_max_ctl = add_numeric_box("input max",
      &c_range_normalize_routine::input_max,
      &c_range_normalize_routine::set_input_max);

  output_min_ctl = add_numeric_box("output min",
      &c_range_normalize_routine::output_min,
      &c_range_normalize_routine::set_output_min);

  output_max_ctl = add_numeric_box("output max",
      &c_range_normalize_routine::output_max,
      &c_range_normalize_routine::set_output_max);

  updateControls();
}

void QRangeNormalizeSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {

    auto_input_range_ctl->setChecked(routine_->auto_input_range());
    input_min_ctl->setValue(routine_->input_min());
    input_max_ctl->setValue(routine_->input_max());
    output_min_ctl->setValue(routine_->output_min());
    output_max_ctl->setValue(routine_->output_max());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}



