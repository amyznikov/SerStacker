/*
 * QImageStackingInputOptions.cc
 *
 *  Created on: Jul 20, 2021
 *      Author: amyznikov
 */

#include "QImageStackingInputOptions.h"
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

QImageStackingInputOptions::QImageStackingInputOptions(QWidget * parent)
  : Base("QImageStackingInputOptions", parent)
{

  enable_remove_bad_pixels_ctl = add_checkbox("Detect Bad Pixels",
      [this](int state) {
        if ( options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != options_->remove_bad_pixels ) {
            options_->remove_bad_pixels = checked;
            emit parameterChanged();
          }
        }
      });

  bad_pixels_variation_threshold_ctl =
      add_numeric_box<double>("Bad Pixels Variation Threshold",
          [this](double v) {
            if ( options_ && options_->bad_pixels_variation_threshold != v ) {
              options_->bad_pixels_variation_threshold = v;
              emit parameterChanged();
            }
          });

  enable_color_maxtrix_ctl = add_checkbox("Apply color matrix if available",
      [this](int state) {
        if ( options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != options_->enable_color_maxtrix ) {
            options_->enable_color_maxtrix = checked;
            emit parameterChanged();
          }
        }
      });

  anscombe_ctl = add_enum_combobox<QAnscombeMethodCombo>(
      "Anscombe Transform:",
      [this](anscombe_method v) {
        if ( options_ && v != options_->anscombe ) {
          options_->anscombe = v;
          emit parameterChanged();
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
          emit applyInputOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);

}

void QImageStackingInputOptions::set_input_options(c_input_options * options)
{
  options_ = options;
  updateControls();
}

c_input_options * QImageStackingInputOptions::input_options() const
{
  return options_;
}

void QImageStackingInputOptions::onupdatecontrols()
{
  if ( !options_) {
    setEnabled(false);
  }
  else {

    enable_remove_bad_pixels_ctl->setChecked(options_->remove_bad_pixels);
    bad_pixels_variation_threshold_ctl->setValue(options_->bad_pixels_variation_threshold);
    enable_color_maxtrix_ctl->setChecked(options_->enable_color_maxtrix);
    anscombe_ctl->setCurrentItem(options_->anscombe);

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
