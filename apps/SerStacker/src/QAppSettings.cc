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

  setWindowTitle("Generic Input Options");
  QVBoxLayout * vbox = new QVBoxLayout(this);
  vbox->addWidget(appSettingsWidget_ = new QGeneralAppSettingsWidget(this));
}

void QGeneralAppSettingsDialogBox::setInputSequenceView(QInputSequenceView * sequenceView)
{
  appSettingsWidget_->setInputSequenceView(sequenceView);
}

QInputSequenceView * QGeneralAppSettingsDialogBox::inputSequenceView() const
{
  return appSettingsWidget_->inputSequenceView();
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
            if ( sequenceView_ ) {
              DEBAYER_ALGORITHM algo = sequenceView_->debayerAlgorithm();
              if ( algo != v ) {
                sequenceView_->setDebayerAlgorithm(v);
              }
            }
          });

  dropBadPixels_ctl =
      add_checkbox("Drop Bad pixels",
          "Set TRUE for bad pixel detection and filtering ",
          [this](bool checked) {
            if ( sequenceView_ && sequenceView_->dropBadPixels() != checked) {
              sequenceView_->setDropBadPixels(checked);
            }
          });

  badPixelsVariationThreshold_ctl =
      add_numeric_box<double>("Bad pixels variation threshold",
          "",
          [this](double v) {
            if ( sequenceView_ && sequenceView_->badPixelsVariationThreshold() != v) {
              sequenceView_->setBadPixelsVariationThreshold(v);
            }
          });

  sourceOutputType_ctl_ =
      add_enum_combobox<c_input_source::OUTPUT_TYPE>("VLO OUTPUT TYPE:",
          "",
          [this](c_input_source::OUTPUT_TYPE v) {
            if ( sequenceView_ ) {
              if ( v != sequenceView_->sourceOutputType()) {
                sequenceView_->setSourceOutputType(v);
              }
            }
          });


#if HAVE_VLO_FILE

  vloDataChannel_ctl_ =
      add_enum_combobox<c_vlo_file::DATA_CHANNEL>("VLO DATA CHANNEL:",
          "",
          [this](c_vlo_file::DATA_CHANNEL v) {
            if ( sequenceView_ ) {
              c_vlo_file::DATA_CHANNEL channel = sequenceView_->vloDataChannel();
              if ( channel != v ) {
                sequenceView_->setVloDataChannel(v);
              }
            }
          });

  applyGhostFilter_ctl =
      add_checkbox("Apply Ghost Filter",
          "Set TRUE to apply Ghost Filter based on doubled echos",
          [this](bool checked) {
            if ( sequenceView_ && sequenceView_->applyGhostFilter() != checked) {
              sequenceView_->setApplyGhostFilter(checked);
            }
          });

#endif

  updateControls();
}

void QGeneralAppSettingsWidget::setInputSequenceView(QInputSequenceView * sequenceView)
{
  if( sequenceView_ ) {
    sequenceView_->disconnect(this);
  }

  if( (sequenceView_ = sequenceView) ) {

    connect(sequenceView_, &QInputSequenceView::debayerAlgorithmChanged,
        [this]() {
          c_update_controls_lock lock(this);
          editorDebayer_ctl->setValue(sequenceView_->debayerAlgorithm());
        });

    connect(sequenceView_, &QInputSequenceView::dropBadPixelsChanged,
        [this]() {
          c_update_controls_lock lock(this);
          dropBadPixels_ctl->setChecked(sequenceView_->dropBadPixels());
        });

    connect(sequenceView_, &QInputSequenceView::badPixelsVariationThresholdChanged,
        [this]() {
          c_update_controls_lock lock(this);
          badPixelsVariationThreshold_ctl->setValue(sequenceView_->badPixelsVariationThreshold());
        });

    connect(sequenceView_, &QInputSequenceView::sourceOutputTypeChanged,
        [this]() {
          c_update_controls_lock lock(this);
          sourceOutputType_ctl_->setValue(sequenceView_->sourceOutputType());
        });

#if HAVE_VLO_FILE
    connect(sequenceView_, &QInputSequenceView::vloDataChannelChanged,
        [this]() {
          c_update_controls_lock lock(this);
          vloDataChannel_ctl_->setValue(sequenceView_->vloDataChannel());
          applyGhostFilter_ctl->setChecked(sequenceView_->applyGhostFilter());
        });
#endif

  }

  updateControls();
}

QInputSequenceView * QGeneralAppSettingsWidget::inputSequenceView() const
{
  return sequenceView_;
}

void QGeneralAppSettingsWidget::onupdatecontrols()
{
  debayer_ctl->setValue(default_debayer_algorithm());

  if ( !sequenceView_ ) {
    editorDebayer_ctl->setEnabled(false);
    dropBadPixels_ctl->setEnabled(false);
    badPixelsVariationThreshold_ctl->setEnabled(false);
#if HAVE_VLO_FILE
    vloDataChannel_ctl_->setEnabled(false);
#endif
  }
  else {
    editorDebayer_ctl->setValue(sequenceView_->debayerAlgorithm());
    dropBadPixels_ctl->setChecked(sequenceView_->dropBadPixels());
    badPixelsVariationThreshold_ctl->setValue(sequenceView_->badPixelsVariationThreshold());

    editorDebayer_ctl->setEnabled(true);
    dropBadPixels_ctl->setEnabled(true);
    badPixelsVariationThreshold_ctl->setEnabled(true);

#if HAVE_VLO_FILE
    vloDataChannel_ctl_->setValue(sequenceView_->vloDataChannel());
    vloDataChannel_ctl_->setEnabled(true);

    applyGhostFilter_ctl->setChecked(sequenceView_->applyGhostFilter());
    applyGhostFilter_ctl->setEnabled(true);
#endif

  }
}

void QGeneralAppSettingsWidget::onload(QSettings & settings)
{
  Base::onload(settings);
}



} // namespace serstacker

