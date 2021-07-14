/*
 * QFrameAccumulationOptions.cc
 *
 *  Created on: Jul 13, 2021
 *      Author: amyznikov
 */

#include "QFrameAccumulationOptions.h"
#include <gui/widgets/addctrl.h>
#include <core/debug.h>


#define ICON_check_all      "check_all"

static const char borderless_style[] = ""
    "QToolButton { border: none; } "
    "QToolButton::menu-indicator { image: none; }"
    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}


QString toString(enum frame_upscale_option v)
{
  return toStdString(v).c_str();
}

enum frame_upscale_option fromString(const QString  & s, enum frame_upscale_option defval )
{
  return fromStdString(s.toStdString(), defval);
}

QString toString(enum frame_accumulation_method v)
{
  return toStdString(v).c_str();
}

enum frame_accumulation_method fromString(const QString  & s, enum frame_accumulation_method defval)
{
  return fromStdString(s.toStdString(), defval);
}

QFrameAccumulationOptions::QFrameAccumulationOptions(QWidget * parent)
  : Base("QFrameAccumulationOptions", parent)
{

  add_combobox(form, "Accumulation Method:",
      accumulation_method_ctl = new QFrameAccumulationMethodCombo(this),
      [this]() {
        if ( options_ && !updatingControls() ) {
          if ( accumulation_method_ctl->currentItem() != options_->accumulation_method ) {
            options_->accumulation_method = accumulation_method_ctl->currentItem();
            emit parameterChanged();
          }
        }
      });

  add_combobox(form, "Upscale:",
      upscale_option_ctl = new QFrameUpscaleOptionCombo(this),
      [this]() {
        if ( options_ && !updatingControls() ) {
          if ( upscale_option_ctl->currentItem() != options_->upscale_option ) {
            options_->upscale_option = upscale_option_ctl->currentItem();
            emit parameterChanged();
          }
        }
      });

  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyFrameAccumulationOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);

  setEnabled(false);
}


void QFrameAccumulationOptions::set_accumulation_options(c_frame_accumulation_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_frame_accumulation_options * QFrameAccumulationOptions::accumulation_options() const
{
  return this->options_;
}

void QFrameAccumulationOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    accumulation_method_ctl->setCurrentItem(options_->accumulation_method);
    upscale_option_ctl->setCurrentItem(options_->upscale_option);

    setEnabled(true);
  }

}
