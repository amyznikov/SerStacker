/*
 * QAppSettings.cc
 *
 *  Created on: Nov 6, 2022
 *      Author: amyznikov
 */

#include "QAppSettings.h"

namespace qserstacker {


QGeneralAppSettingsDialogBox::QGeneralAppSettingsDialogBox(QWidget * parent) :
  Base(parent)
{
  QVBoxLayout * vbox = new QVBoxLayout(this);
  vbox->addWidget(appSettingsWidget_ = new QGeneralAppSettingsWidget(this));
}

void QGeneralAppSettingsDialogBox::setImageEditor(QImageEditor * imageEditor)
{
  appSettingsWidget_->setImageEditor(imageEditor);
}

QImageEditor * QGeneralAppSettingsDialogBox::imageEditor() const
{
  return appSettingsWidget_->imageEditor();
}

void QGeneralAppSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGeneralAppSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}



QGeneralAppSettingsWidget::QGeneralAppSettingsWidget(QWidget * parent) :
    Base("QGeneralAppSettingsWidget", parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>(
          "Stacker debayer algoritm:",
          [this](DEBAYER_ALGORITHM v) {
              if ( v != default_debayer_algorithm() ) {
                set_default_debayer_algorithm(v);
              }
          });

  editor_debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>(
          "Editor debayer algoritm:",
          [this](DEBAYER_ALGORITHM v) {
            if ( imageEditor_ ) {
              DEBAYER_ALGORITHM algo = imageEditor_->debayerAlgorithm();
              if ( algo != v ) {
                imageEditor_->setDebayerAlgorithm(v);
              }

            }
          });

  updateControls();
}

void QGeneralAppSettingsWidget::setImageEditor(QImageEditor * imageEditor)
{
  if (imageEditor_ ) {
    disconnect(imageEditor_);
  }

  if( (imageEditor_ = imageEditor) ) {
    connect(imageEditor_, &QImageEditor::debayerAlgorithmChanged,
        [this]() {
          c_update_controls_lock lock(this);
          editor_debayer_ctl->setValue(imageEditor_->debayerAlgorithm());
        });
  }

  updateControls();
}

QImageEditor * QGeneralAppSettingsWidget::imageEditor() const
{
  return imageEditor_;
}

void QGeneralAppSettingsWidget::onupdatecontrols()
{
  debayer_ctl->setValue(default_debayer_algorithm());

  if ( !imageEditor_ ) {
    editor_debayer_ctl->setEnabled(false);
  }
  else {
    editor_debayer_ctl->setValue(imageEditor_->debayerAlgorithm());
    editor_debayer_ctl->setEnabled(true);
  }
}

void QGeneralAppSettingsWidget::onload(QSettings & settings)
{
  Base::onload(settings);
}



} // namespace qserstacker

