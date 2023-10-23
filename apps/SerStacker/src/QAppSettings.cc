/*
 * QAppSettings.cc
 *
 *  Created on: Nov 6, 2022
 *      Author: amyznikov
 */

#include "QAppSettings.h"

namespace serstacker {


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
      add_enum_combobox<DEBAYER_ALGORITHM>("Stacker debayer algorithm:",
          "",
          [this](DEBAYER_ALGORITHM v) {
              if ( v != default_debayer_algorithm() ) {
                set_default_debayer_algorithm(v);
              }
          });

  editorDebayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Editor debayer algorithm:",
          "",
          [this](DEBAYER_ALGORITHM v) {
            if ( imageEditor_ ) {
              DEBAYER_ALGORITHM algo = imageEditor_->debayerAlgorithm();
              if ( algo != v ) {
                imageEditor_->setDebayerAlgorithm(v);
              }
            }
          });

  dropBadPixels_ctl =
      add_checkbox("Drop Bad pixels",
          "Set TRUE for bad pixel detection and filtering ",
          [this](bool checked) {
            if ( imageEditor_ && imageEditor_->dropBadPixels() != checked) {
              imageEditor_->setDropBadPixels(checked);
            }
          });

  badPixelsVariationThreshold_ctl =
      add_numeric_box<double>("Bad pixels variation threshold",
          "",
          [this](double v) {
            if ( imageEditor_ && imageEditor_->badPixelsVariationThreshold() != v) {
              imageEditor_->setBadPixelsVariationThreshold(v);
            }
          });

#if HAVE_VLO_FILE
  vloDataChannel_ctl_ =
      add_enum_combobox<c_vlo_file::DATA_CHANNEL>("VLO DATA CHANNEL:",
          "",
          [this](c_vlo_file::DATA_CHANNEL v) {
            if ( imageEditor_ ) {
              c_vlo_file::DATA_CHANNEL channel = imageEditor_->vloDataChannel();
              if ( channel != v ) {
                imageEditor_->setVloDataChannel(v);
              }
            }
          });
#endif

  updateControls();
}

void QGeneralAppSettingsWidget::setImageEditor(QImageEditor * imageEditor)
{
  if( imageEditor_ ) {
    imageEditor_->disconnect(this);
  }

  if( (imageEditor_ = imageEditor) ) {

    connect(imageEditor_, &QImageEditor::debayerAlgorithmChanged,
        [this]() {
          c_update_controls_lock lock(this);
          editorDebayer_ctl->setValue(imageEditor_->debayerAlgorithm());
        });

    connect(imageEditor_, &QImageEditor::dropBadPixelsChanged,
        [this]() {
          c_update_controls_lock lock(this);
          dropBadPixels_ctl->setChecked(imageEditor_->dropBadPixels());
        });

    connect(imageEditor_, &QImageEditor::badPixelsVariationThresholdChanged,
        [this]() {
          c_update_controls_lock lock(this);
          badPixelsVariationThreshold_ctl->setValue(imageEditor_->badPixelsVariationThreshold());
        });

#if HAVE_VLO_FILE
    connect(imageEditor_, &QImageEditor::vloDataChannelChanged,
        [this]() {
          c_update_controls_lock lock(this);
          vloDataChannel_ctl_->setValue(imageEditor_->vloDataChannel());
        });
#endif

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
    editorDebayer_ctl->setEnabled(false);
    dropBadPixels_ctl->setEnabled(false);
    badPixelsVariationThreshold_ctl->setEnabled(false);
#if HAVE_VLO_FILE
    vloDataChannel_ctl_->setEnabled(false);
#endif
  }
  else {
    editorDebayer_ctl->setValue(imageEditor_->debayerAlgorithm());
    dropBadPixels_ctl->setChecked(imageEditor_->dropBadPixels());
    badPixelsVariationThreshold_ctl->setValue(imageEditor_->badPixelsVariationThreshold());

    editorDebayer_ctl->setEnabled(true);
    dropBadPixels_ctl->setEnabled(true);
    badPixelsVariationThreshold_ctl->setEnabled(true);

#if HAVE_VLO_FILE
    vloDataChannel_ctl_->setValue(imageEditor_->vloDataChannel());
    vloDataChannel_ctl_->setEnabled(true);
#endif

  }
}

void QGeneralAppSettingsWidget::onload(QSettings & settings)
{
  Base::onload(settings);
}



} // namespace serstacker

