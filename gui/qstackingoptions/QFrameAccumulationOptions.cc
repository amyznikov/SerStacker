/*
 * QFrameAccumulationOptions.cc
 *
 *  Created on: Jul 13, 2021
 *      Author: amyznikov
 */

#include "QFrameAccumulationOptions.h"
#include <core/debug.h>


#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}

QFrameAccumulationOptions::QFrameAccumulationOptions(QWidget * parent) :
    Base("QFrameAccumulationOptions", parent)
{

  accumulation_method_ctl = add_enum_combobox<frame_accumulation_method>(
      "Accumulation Method:",
      [this](frame_accumulation_method v) {
        if ( options_ && v != options_->accumulation_method ) {
          options_->accumulation_method = v;
          emit parameterChanged();
        }
      });

  lksize_ctl =
      add_numeric_box<int>("lksize:",
          [this](int v) {
            if ( options_ && v != options_->lksize ) {
              options_->lksize = v;
              emit parameterChanged();
            }
          });

  scale_size_ctl =
      add_numeric_box<int>("scale:",
          [this](int v) {
            if ( options_ && v != options_->scale_size ) {
              options_->scale_size = v;
              emit parameterChanged();
            }
          });

  minv_ctl =
      add_numeric_box<double>("miv:",
          [this](double v) {
            if ( options_ && v != options_->minv ) {
              options_->minv = v;
              emit parameterChanged();
            }
          });

  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
//  applyToAll_ctl->setStyleSheet(borderless_style);
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
    lksize_ctl->setValue(options_->lksize);
    scale_size_ctl->setValue(options_->scale_size);
    minv_ctl->setValue(options_->minv);

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
