/*
 * QAlignColorChannelsSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QAlignColorChannelsSettings.h"

const QAlignColorChannelsSettings::ClassFactory QAlignColorChannelsSettings::classFactory;

QAlignColorChannelsSettings::QAlignColorChannelsSettings(const c_align_color_channels_routine::ptr & routine, QWidget * parent) :
    Base(&classFactory, routine, parent)
{
  reference_channel_ctl = add_numeric_box("Reference channel:",
      &c_align_color_channels_routine::reference_channel,
      &c_align_color_channels_routine::set_reference_channel);

  enable_threshold_ctl = add_checkbox("Threshold:",
      &c_align_color_channels_routine::enable_threshold,
      &c_align_color_channels_routine::set_enable_threshold);

  threshold_ctl = add_numeric_box("Threshold value:",
      &c_align_color_channels_routine::threshold,
      &c_align_color_channels_routine::set_threshold);

  updateControls();
}

void QAlignColorChannelsSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    enable_threshold_ctl->setChecked(routine_->enable_threshold());
    reference_channel_ctl->setValue(routine_->reference_channel());
    threshold_ctl->setValue(routine_->threshold());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
