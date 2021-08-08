/*
 * QROISelectionOptions.cc
 *
 *  Created on: Jul 17, 2021
 *      Author: amyznikov
 */

#include "QROISelectionOptions.h"
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


QString toString(enum roi_selection_method v)
{
  return QString::fromStdString(toStdString(v));
}

enum roi_selection_method fromString(const QString  & s, enum roi_selection_method defval )
{
  return fromStdString(s.toStdString(), defval);
}


QROISelectionOptions::QROISelectionOptions(QWidget * parent)
  : Base("QROISelectionOptions", parent)
{

  selectionMethod_ctl = add_enum_combobox<QROISelectionMethodCombo>(
      "Detect feature:",
      [this](roi_selection_method v) {
        if ( options_ && v != options_->method ) {
          options_->method = v;
          emit parameterChanged();
        }
      });

  cropSize_ctl =
      add_numeric_box(form, "Crop Size WxH:",
          [this]() {
            if ( options_ && !updatingControls() ) {
              cv::Size v;
              if ( fromString(cropSize_ctl->text(), &v) && v != options_->crop_size ) {
                options_->crop_size = v;
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
          emit applyROISelectionOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);

  setEnabled(false);

  updateControls();
}

void QROISelectionOptions::set_roi_selection_options(c_roi_selection_options * options)
{
  options_ = options;
  updateControls();
}

const c_roi_selection_options * QROISelectionOptions::roi_selection_options() const
{
  return options_;
}


void QROISelectionOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    selectionMethod_ctl->setCurrentItem(options_->method);
    cropSize_ctl->setValue(options_->crop_size);

    setEnabled(true);
  }

}
