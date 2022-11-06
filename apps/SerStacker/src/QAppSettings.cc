/*
 * QAppSettings.cc
 *
 *  Created on: Nov 6, 2022
 *      Author: amyznikov
 */

#include "QAppSettings.h"


QGeneralAppSettingsDialogBox::QGeneralAppSettingsDialogBox(QWidget * parent) :
  Base(parent)
{
  QVBoxLayout * vbox = new QVBoxLayout(this);
  vbox->addWidget(appSettingsWidget_ = new QGeneralAppSettingsWidget(this));
}


QGeneralAppSettingsWidget::QGeneralAppSettingsWidget(QWidget * parent) :
    Base("QGeneralAppSettingsWidget", parent)
{

  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>(
          "Default debayer algoritm",
          [this](DEBAYER_ALGORITHM v) {
              if ( v != default_debayer_algorithm() ) {
                set_default_debayer_algorithm(v);
              }
          });

  updateControls();
}

void QGeneralAppSettingsWidget::onupdatecontrols()
{
  debayer_ctl->setValue(default_debayer_algorithm());
}

void QGeneralAppSettingsWidget::onload(QSettings & settings)
{
  Base::onload(settings);
}

