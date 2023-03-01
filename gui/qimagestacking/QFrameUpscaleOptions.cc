/*
 * QFrameUpscaleOptions.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "QFrameUpscaleOptions.h"

QFrameUpscaleOptions::QFrameUpscaleOptions(QWidget * parent) :
    Base("QFrameUpscaleOptions", parent)
{

  upscale_option_ctl =
      add_enum_combobox<frame_upscale_option>(
          "Upscale:",
          [this](frame_upscale_option v) {
            if ( options_ && v != options_->upscale_option ) {
              options_->upscale_option = upscale_option_ctl->currentItem();
              Q_EMIT parameterChanged();
            }
          });

  upscale_stage_ctl =
      add_enum_combobox<frame_upscale_stage>(
          "Stage:",
          [this](frame_upscale_stage v) {
            if ( options_ && v != options_->upscale_stage ) {
              options_->upscale_stage = upscale_stage_ctl->currentItem();
              Q_EMIT parameterChanged();
            }
          });

  setEnabled(false);

}

void QFrameUpscaleOptions::set_upscale_options(c_frame_upscale_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_frame_upscale_options * QFrameUpscaleOptions::upscale_options() const
{
  return this->options_;
}

void QFrameUpscaleOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    upscale_option_ctl->setCurrentItem(options_->upscale_option);
    upscale_stage_ctl->setCurrentItem(options_->upscale_stage);

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
