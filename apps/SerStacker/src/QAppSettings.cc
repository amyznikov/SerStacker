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

void QGeneralAppSettingsDialogBox::setInputSourceView(QInputSourceView * sequenceView)
{
  appSettingsWidget_->setInputSourceView(sequenceView);
}

QInputSourceView * QGeneralAppSettingsDialogBox::inputSourceView() const
{
  return appSettingsWidget_->inputSourceView();
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

///////////////////////////////////////////////////////////////////////

QGeneralAppInputSettings::QGeneralAppInputSettings(QWidget * parent) :
    Base("", parent)
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

//  sourceOutputType_ctl_ =
//      add_enum_combobox<c_input_source::OUTPUT_TYPE>("OUTPUT TYPE:",
//          "",
//          [this](c_input_source::OUTPUT_TYPE v) {
//            if ( sequenceView_ ) {
//              if ( v != sequenceView_->sourceOutputType()) {
//                sequenceView_->setSourceOutputType(v);
//              }
//            }
//          });

  updateControls();
}

void QGeneralAppInputSettings::setInputSourceView(QInputSourceView * sequenceView)
{
  if( sequenceView_ ) {
    sequenceView_->disconnect(this);
  }

  if( (sequenceView_ = sequenceView) ) {

    connect(sequenceView_, &QInputSourceView::debayerAlgorithmChanged,
        [this]() {
          c_update_controls_lock lock(this);
          editorDebayer_ctl->setValue(sequenceView_->debayerAlgorithm());
        });

    connect(sequenceView_, &QInputSourceView::dropBadPixelsChanged,
        [this]() {
          c_update_controls_lock lock(this);
          dropBadPixels_ctl->setChecked(sequenceView_->dropBadPixels());
        });

    connect(sequenceView_, &QInputSourceView::badPixelsVariationThresholdChanged,
        [this]() {
          c_update_controls_lock lock(this);
          badPixelsVariationThreshold_ctl->setValue(sequenceView_->badPixelsVariationThreshold());
        });

//    connect(sequenceView_, &QInputSourceView::sourceOutputTypeChanged,
//        [this]() {
//          c_update_controls_lock lock(this);
//          sourceOutputType_ctl_->setValue(sequenceView_->sourceOutputType());
//        });
  }

  updateControls();
}

QInputSourceView * QGeneralAppInputSettings::inputSourceView() const
{
  return sequenceView_;
}


void QGeneralAppInputSettings::onload(QSettings & settings)
{
  Base::onload(settings);

}

void QGeneralAppInputSettings::onupdatecontrols()
{
  debayer_ctl->setValue(default_debayer_algorithm());

  if ( !sequenceView_ ) {
    editorDebayer_ctl->setEnabled(false);
    dropBadPixels_ctl->setEnabled(false);
    badPixelsVariationThreshold_ctl->setEnabled(false);
  }
  else {
    editorDebayer_ctl->setValue(sequenceView_->debayerAlgorithm());
    dropBadPixels_ctl->setChecked(sequenceView_->dropBadPixels());
    badPixelsVariationThreshold_ctl->setValue(sequenceView_->badPixelsVariationThreshold());

    editorDebayer_ctl->setEnabled(true);
    dropBadPixels_ctl->setEnabled(true);
    badPixelsVariationThreshold_ctl->setEnabled(true);
  }

}

//////////////////////////////////////////////////////////////////////

QVLOGhostFilterSettings::QVLOGhostFilterSettings(QWidget * parent) :
    Base("", parent)
{

  enableGhostFilter_ctl =
      add_checkbox("Apply Ghost Filter",
          "Set TRUE to apply Ghost Filter based on doubled echos",
          [this](bool checked) {
//            if ( sequenceView_ && sequenceView_->vlo_processing_options()->ghost_filter.enabled != checked) {
//              sequenceView_->vlo_processing_options()->ghost_filter.enabled = checked;
//              sequenceView_->update_as_vlo_processing_options_chaned();
//            }
          });

  saturation_level_ctl  =
      add_numeric_box<double>("Area saturation level:",
          "",
          [this](double v) {
            if ( sequenceView_ ) {
//              if ( sequenceView_->vlo_processing_options()->ghost_filter.saturation_level != v) {
//                sequenceView_->vlo_processing_options()->ghost_filter.saturation_level = v;
//                sequenceView_->update_as_vlo_processing_options_chaned();
//              }
            }
          });


  doubled_distanse_systematic_correction_ctl  =
      add_numeric_box<double>("Systematic error correction:",
          "",
          [this](double v) {
            if ( sequenceView_ ) {
//              if ( sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_systematic_correction != v) {
//                sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_systematic_correction = v;
//                sequenceView_->update_as_vlo_processing_options_chaned();
//              }
            }
          });

    doubled_distanse_depth_tolerance_ctl  =
        add_numeric_box<double>("Depth tolerance:",
            "",
            [this](double v) {
              if ( sequenceView_ ) {
//                if ( sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_depth_tolerance != v) {
//                  sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_depth_tolerance = v;
//                  sequenceView_->update_as_vlo_processing_options_chaned();
//                }
              }
            });

    updateControls();
}

void QVLOGhostFilterSettings::setInputSourceView(QInputSourceView * sequenceView)
{
  if( sequenceView_ ) {
    sequenceView_->disconnect(this);
  }

  if( (sequenceView_ = sequenceView) ) {
  }

  updateControls();
}

QInputSourceView * QVLOGhostFilterSettings::inputSourceView() const
{
  return sequenceView_;
}

void QVLOGhostFilterSettings::onload(QSettings & settings)
{

}

void QVLOGhostFilterSettings::onupdatecontrols()
{
  if ( !sequenceView_ ) {
    setEnabled(false);
  }
  else {
//    enableGhostFilter_ctl->setChecked(sequenceView_->vlo_processing_options()->ghost_filter.enabled);
//    saturation_level_ctl ->setValue(sequenceView_->vlo_processing_options()->ghost_filter.saturation_level);
//    doubled_distanse_systematic_correction_ctl ->setValue(sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_systematic_correction);
//    doubled_distanse_depth_tolerance_ctl ->setValue(sequenceView_->vlo_processing_options()->ghost_filter.doubled_distanse_depth_tolerance);

    setEnabled(true);
  }

}

//////////////////////////////////////////////////////////////////////

QVLOLowIntensityFilterSettings::QVLOLowIntensityFilterSettings(QWidget * parent) :
    Base("", parent)
{

  enabled_ctl =
      add_checkbox("Ebable Low intensity filter",
          "",
          [this](bool checked) {
//            if ( sequenceView_ && sequenceView_->vlo_processing_options()->low_intensity_filter.enabled != checked ) {
//              sequenceView_->vlo_processing_options()->low_intensity_filter.enabled = checked;
//              sequenceView_->update_as_vlo_processing_options_chaned();
//            }
          },
          [this](bool * checked) {
            if ( sequenceView_ ) {
//              * checked = sequenceView_->vlo_processing_options()->low_intensity_filter.enabled;
              return true;
            }
            return false;
          });

  low_intensity_level_ctl =
      add_numeric_box<double>("low_intensity_threshold",
          "",
          [this](
              double v) {
//                if ( sequenceView_ && sequenceView_->vlo_processing_options()->low_intensity_filter.low_intensity_level != v ) {
//                  sequenceView_->vlo_processing_options()->low_intensity_filter.low_intensity_level = v;
//                  sequenceView_->update_as_vlo_processing_options_chaned();
//                }

              },
          [this](double * v) {
            if ( sequenceView_ ) {
//              * v = sequenceView_->vlo_processing_options()->low_intensity_filter.low_intensity_level;
              return true;
            }
            return false;
          });


  u_ctl =
      add_numeric_box<double>("U",
          "",
          [this](
              double v) {
//                if ( sequenceView_ && sequenceView_->vlo_processing_options()->low_intensity_filter.u != v ) {
//                  sequenceView_->vlo_processing_options()->low_intensity_filter.u = v;
//                  sequenceView_->update_as_vlo_processing_options_chaned();
//                }

              },
          [this](double * v) {
            if ( sequenceView_ ) {
//              * v = sequenceView_->vlo_processing_options()->low_intensity_filter.u;
              return true;
            }
            return false;
          });


  v_ctl =
      add_numeric_box<double>("V",
          "",
          [this](
              double v) {
//                if ( sequenceView_ && sequenceView_->vlo_processing_options()->low_intensity_filter.v != v ) {
//                  sequenceView_->vlo_processing_options()->low_intensity_filter.v = v;
//                  sequenceView_->update_as_vlo_processing_options_chaned();
//                }

              },
          [this](double * v) {
            if ( sequenceView_ ) {
//              * v = sequenceView_->vlo_processing_options()->low_intensity_filter.v;
              return true;
            }
            return false;
          });


  updateControls();
}

void QVLOLowIntensityFilterSettings::setInputSourceView(QInputSourceView * sequenceView)
{
  if( sequenceView_ ) {
    sequenceView_->disconnect(this);
  }

  if( (sequenceView_ = sequenceView) ) {
  }

  updateControls();
}

QInputSourceView * QVLOLowIntensityFilterSettings::inputSourceView() const
{
  return sequenceView_;
}

void QVLOLowIntensityFilterSettings::onload(QSettings & settings)
{
}

void QVLOLowIntensityFilterSettings::onupdatecontrols()
{
  if ( !sequenceView_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }

}


//////////////////////////////////////////////////////////////////////
QVLOInputSettings::QVLOInputSettings(QWidget * parent) :
    Base("", parent)
{
  vloDataChannel_ctl_ =
      add_enum_combobox<VLO_DATA_CHANNEL>("VLO DATA CHANNEL:",
          "",
          [this](VLO_DATA_CHANNEL v) {
            if ( sequenceView_ ) {
              VLO_DATA_CHANNEL channel = sequenceView_->vloDataChannel();
              if ( channel != v ) {
                sequenceView_->setVloDataChannel(v);
              }
            }
          },
          [this](VLO_DATA_CHANNEL * v) {
            if ( sequenceView_ ) {
              *v = sequenceView_->vloDataChannel();
              return true;
            }
            return false;
          }
          );
  addRow(tab_ctl = new QTabWidget(this));
  tab_ctl->addTab(lowIntensityFilter_ctl = new QVLOLowIntensityFilterSettings(this), "Low Intensity Filter");
  tab_ctl->addTab(ghostFilter_ctl = new QVLOGhostFilterSettings(this), "Ghost Filter");

  updateControls();
}

void QVLOInputSettings::setInputSourceView(QInputSourceView * sequenceView)
{
  if( sequenceView_ ) {
    sequenceView_->disconnect(this);
  }

  if( (sequenceView_ = sequenceView) ) {

    ghostFilter_ctl->setInputSourceView(sequenceView);
    lowIntensityFilter_ctl->setInputSourceView(sequenceView);

    connect(sequenceView_, &QInputSourceView::vloDataChannelChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();

}

QInputSourceView * QVLOInputSettings::inputSourceView() const
{
  return sequenceView_;
}

void QVLOInputSettings::onload(QSettings & settings)
{
  Base::onload(settings);

}

void QVLOInputSettings::onupdatecontrols()
{
  if ( !sequenceView_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

QGeneralAppSettingsWidget::QGeneralAppSettingsWidget(QWidget * parent) :
    Base(parent)
{

  QVBoxLayout * layout_ = new QVBoxLayout(this);

  layout_->addWidget(tab_ctl = new QTabWidget(this));

  tab_ctl->addTab(genericInputSettings_ctl = new QGeneralAppInputSettings(this),
      QIcon(), "Generic Input Settings");

  tab_ctl->addTab(vloSettings_ctl = new QVLOInputSettings(this),
      QIcon(), "VLO");

}

void QGeneralAppSettingsWidget::setInputSourceView(QInputSourceView * sequenceView)
{
  genericInputSettings_ctl->setInputSourceView(sequenceView);
  vloSettings_ctl->setInputSourceView(sequenceView);
}

QInputSourceView * QGeneralAppSettingsWidget::inputSourceView() const
{
  return genericInputSettings_ctl->inputSourceView();
}



} // namespace serstacker

