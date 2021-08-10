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

  motion_ctl = add_combobox<QMotionTypeCombo>("Motion type:",
      &c_align_color_channels_routine::motion_type,
      &c_align_color_channels_routine::set_motion_type);

  interpolation_ctl = add_combobox<QInterpolationTypeCombo>("Interpolation:",
      &c_align_color_channels_routine::interpolation,
      &c_align_color_channels_routine::set_interpolation);

  eps_ctl = add_numeric_box("eps:",
      &c_align_color_channels_routine::eps,
      &c_align_color_channels_routine::set_eps);

  max_iterations_ctl = add_numeric_box("max_iterations:",
      &c_align_color_channels_routine::max_iterations,
      &c_align_color_channels_routine::set_max_iterations);

  smooth_sigma_ctl = add_numeric_box("smooth_sigma:",
      &c_align_color_channels_routine::smooth_sigma,
      &c_align_color_channels_routine::set_smooth_sigma);

  update_step_scale_ctl = add_numeric_box("update_step_scale:",
      &c_align_color_channels_routine::update_step_scale,
      &c_align_color_channels_routine::set_update_step_scale);

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

    motion_ctl->setCurrentItem(routine_->motion_type());
    interpolation_ctl->setCurrentItem(routine_->interpolation());
    eps_ctl->setValue(routine_->eps());
    max_iterations_ctl->setValue(routine_->max_iterations());
    smooth_sigma_ctl->setValue(routine_->smooth_sigma());
    update_step_scale_ctl->setValue(routine_->update_step_scale());

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
