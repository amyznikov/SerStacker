/*
 * QFrameUpscaleOptions.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "QFrameUpscaleOptions.h"

//#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
//}

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

//  applyToAll_ctl = new QToolButton(this);
//  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//  applyToAll_ctl->setIconSize(QSize(16,16));
//  //applyToAll_ctl->setStyleSheet(borderless_style);
//  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
//  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
//  connect(applyToAll_ctl, &QToolButton::clicked,
//      [this]() {
//        if ( options_ ) {
//          Q_EMIT applyFrameUpScaleOptionsToAllRequested(*options_);
//        }
//      });
//
//  form->addRow(applyToAll_ctl);

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
