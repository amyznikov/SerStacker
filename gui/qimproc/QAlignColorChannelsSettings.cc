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
  reference_channel_ctl = add_numeric_box("reference channel", &routine_,
      &c_align_color_channels_routine::reference_channel,
      &c_align_color_channels_routine::set_reference_channel);

  updateControls();
}

void QAlignColorChannelsSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    reference_channel_ctl->setValue(routine_->reference_channel());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}
