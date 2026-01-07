/*
 * QLCSCTPCameraControls.cc
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#include "QLCSCTPCameraControls.h"
#include "QLCSCTPStreams.h"

namespace serimager {


QLCSCTPCameraControls::QLCSCTPCameraControls(const QLCSCTPCamera::sptr & camera, QWidget * parent) :
    Base(parent),
    _camera(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  url_ctl =
      add_widget2<QLCSCTPUrlWidget>("URL:",
          &QLCSCTPUrlWidget::urlChanged,
          [this]() {
            if ( _camera ) {
              _camera->setUrl(url_ctl->url());
            }
          },
          [this]() {
            if ( _camera ) {
              url_ctl->setUrl(_camera->url());
            }
          });

  cameras_ctl =
      add_combobox<QComboBox>("Device:",
          "Select specific device",
          false,
          [this](int cursel, QComboBox * combo) {
            if (_camera ) {
              _camera->setSelectedCameraIndex(cursel);
              populateCameraControls();
              populateStreams();
              populateFormats();
              populateSizes();
            }
          }/*,
          [this](int * cursel, QComboBox * ) -> bool {
            if (_camera ) {
              * cursel = _camera->selectedCameraIndex();
              return true;
            }
            return false;
          }*/);

  streams_ctl =
      add_combobox<QComboBox>("Stream:",
          "Select specific Stream (role)",
          false,
          [this](int cursel, QComboBox * combo) {
            if (_camera ) {
              _camera->setSelectedStreamIndex(cursel);
              populateFormats();
              populateSizes();
            }
          }/*,
          [this](int * cursel, QComboBox*) -> bool {
            if (_camera ) {
              * cursel = _camera->selectedStreamIndex();
              return true;
            }
            return false;
          }*/);

  formats_ctl =
      add_combobox<QComboBox>("Format:",
          "Select specific pixel format",
          false,
          [this](int cursel, QComboBox * combo) {
            if (_camera ) {
              _camera->setSelectedFormatIndex(cursel);
              populateSizes();
            }
          }/*,
          [this](int * cursel, QComboBox*) -> bool {
            if (_camera ) {
              * cursel = _camera->selectedFormatIndex();
              return true;
            }
            return false;
          }*/);

  sizes_ctl =
      add_combobox<QComboBox>("Size:",
          "Select specific frame size",
          false,
          [this](int cursel, QComboBox * combo) {
            if (_camera ) {
              _camera->setSelectedSizeIndex(cursel);
            }
          }/*,
          [this](int * cursel, QComboBox*) -> bool {
            if (_camera ) {
              * cursel = _camera->selectedSizeIndex();
              return true;
            }
            return false;
          }*/);

  cameraDeviceBuffers_ctl =
      add_spinbox("Buffers", "Specify max device frame buffers",
          [this](int v) {
            if (_camera ) {
              _camera->setCameraDeviceBuffers(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if (_camera ) {
              * v = _camera->cameraDeviceBuffers();
              return true;
            }
            return false;
          });

  ////////////////////////////////////////////

  AeEnable_ctl = createCheckBoxCameraControl(this, "AeEnable", "AeEnable",
      "Enable or disable the AEGC algorithm.\n"
          "When this control is set to true, both ExposureTimeMode and AnalogueGainMode are set to auto,\n"
          "and if this control is set to false then both are set to manual.");

  ExposureTime_ctl = createSpinBoxCameraControl(this, "ExposureTime", "ExposureTime",
      "Exposure time in microseconds for the frame applied in the sensor device.\n"
          "This control will only take effect if ExposureTimeMode is Manual.\n"
          "If this control is set when ExposureTimeMode is Auto, the value will be ignored and will not be retained."
      );

  AnalogueGain_ctl = createDoubleSpinBoxCameraControl(this, "AnalogueGain", "AnalogueGain",
      "Floating-point Analogue gain value applied in the sensor device.\n"
          "The value of the control specifies the gain multiplier applied to all colour channels.\n"
          "This value cannot be lower than 1.0.\n"
          "This control will only take effect if AnalogueGainMode is Manual.");

  AeExposureMode_ctl = createComboBoxCameraControl(this, "AeExposureMode", "AeExposureMode", "");
  AeMeteringMode_ctl =  createComboBoxCameraControl(this, "AeMeteringMode", "AeMeteringMode", "");
  AeConstraintMode_ctl  = createComboBoxCameraControl(this, "AeConstraintMode", "AeConstraintMode", "");
  AeFlickerMode_ctl  = createComboBoxCameraControl(this, "AeFlickerMode", "AeFlickerMode", "");
  AeFlickerPeriod_ctl = createSpinBoxCameraControl(this, "AeFlickerPeriod", "AeFlickerPeriod", "");

  ////////////////////////////////////////////
  // ISP Color & White Balance

  ISPColorToneGroup_ctl =
      add_expandable_groupbox("ISP Color/Tone Controls",
          ISPColorToneControls_ctl = new QSettingsWidget("", this));

  AwbEnable_ctl = createCheckBoxCameraControl(ISPColorToneControls_ctl, "AwbEnable", "AwbEnable",
      "Enable or disable the AWB.\n"
          "When AWB is enabled, the algorithm estimates the colour temperature of the scene \n"
          "and computes colour gains and the colour correction matrix automatically.\n"
          "The computed colour temperature, gains and correction matrix are reported in metadata.\n"
          "The corresponding controls are ignored if set in a request.\n"
          "When AWB is disabled, the colour temperature, gains and correction matrix are not updated automatically\n"
          "and can be set manually in requests.");

  AwbMode_ctl = createComboBoxCameraControl(ISPColorToneControls_ctl, "AwbMode", "AwbMode",
      "Specify the illuminant to use for the AWB algorithm.");

  ColourTemperature_ctl = createSpinBoxCameraControl(ISPColorToneControls_ctl, "ColourTemperature", "ColourTemperature",
      "Manual Kelvin setting (if supported by IPA).");

//  QNumericBox * ColourGains_ctl = nullptr; //  Manual Red and Blue channel gain values.
//  QNumericBox * ColourCorrectionMatrix_ctl = nullptr; //  Fine-tuning of the color gamut (3x3 matrix).

  ////////////////////////////////////////////
  // ISP Visual Adjustments
  ISPVisualAdjustmentsGroup_ctl =
      add_expandable_groupbox("ISP Visual Adjustments Controls",
          ISPVisualAdjustmentsControls_ctl = new QSettingsWidget("", this));

  Brightness_ctl = createDoubleSpinBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "Brightness", "Brightness", "General pixel offset");
  Contrast_ctl = createDoubleSpinBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "Contrast", "Contrast", "Luminance range scaling");
  Saturation_ctl = createDoubleSpinBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "Saturation", "Saturation", "Color intensity scaling");
  Gamma_ctl = createDoubleSpinBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "Gamma", "Gamma", "Fixed gamma curve application");
  Sharpness_ctl = createDoubleSpinBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "Sharpness", "Sharpness", "Edge enhancement strength");
  NoiseReductionMode_ctl = createComboBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "NoiseReductionMode", "NoiseReductionMode", "Set the level/type of noise filtering");
  HdrMode_ctl = createComboBoxCameraControl(ISPVisualAdjustmentsControls_ctl, "HdrMode", "HdrMode", "Enable High Dynamic Range imaging (e.g., Off, Sensor, Multi-frame)");

  ////////////////////////////////////////////
  //  LensPosition_ctl = createDoubleSpinBoxCameraControl(this, "LensPosition", "LensPosition",
  //      "Set and report the focus lens position.\n"
  //        "This control instructs the lens to move to a particular position\n"
  //        "and also reports back the position of the lens for each frame.\n"
  //        "The LensPosition control is ignored unless the AfMode is set to AfModeManual,\n"
  //        "though the value is reported back unconditionally in all modes.\n"
  //        "This value, which is generally a non-integer, is the reciprocal of the focal distance in metres,\n"
  //        "also known as dioptres."
  //      );
  ////////////////////////////////////////////

  if( _camera ) {

    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    connect(_camera.get(), &QLCSCTPCamera::parametersChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}

QLCSCTPCameraControls::~QLCSCTPCameraControls()
{
}

void QLCSCTPCameraControls::onCameraStateChanged()
{
  updateControls();
}

void QLCSCTPCameraControls::populateCameras()
{
  QSignalBlocker block(cameras_ctl);
  cameras_ctl->clear();
  for ( const auto & cam: _camera->cameras() ) {
    cameras_ctl->addItem(cam.id);
  }
  cameras_ctl->setCurrentIndex(_camera->selectedCameraIndex());
}

void QLCSCTPCameraControls::populateStreams()
{
  QSignalBlocker block(streams_ctl);
  streams_ctl->clear();
  const auto * cam = _camera->selectedCamera();
  if ( cam ) {
    for ( const auto & strm: cam->streams ) {
      streams_ctl->addItem(strm.role);
    }
    streams_ctl->setCurrentIndex(cam->selectedStreamIndex);
  }
}

void QLCSCTPCameraControls::populateFormats()
{
  QSignalBlocker block(formats_ctl);
  formats_ctl->clear();
  const auto * strm = _camera->selectedStream();
  if ( strm ) {
    for ( const auto & fmt: strm->formats ) {
      formats_ctl->addItem(fmt.format);
    }
    formats_ctl->setCurrentIndex(strm->selectedFormatIndex);
  }
}

void QLCSCTPCameraControls::populateSizes()
{
  QSignalBlocker block(sizes_ctl);
  sizes_ctl->clear();
  const auto * fmt = _camera->selectedFormat();
  if ( fmt ) {
    for ( const auto & size: fmt->sizes ) {
      sizes_ctl->addItem(size);
    }
    sizes_ctl->setCurrentIndex(fmt->selectedSizeIndex);
  }
}



void QLCSCTPCameraControls::onupdatecontrols()
{
  if( !_camera ) {
    setEnabled(false);
  }
  else {
    const QImagingCamera::State cameraState = _camera->state();
    if( cameraState == QImagingCamera::State_connected ) {
      populateCameras();
      populateCameraControls();
      populateStreams();
      populateFormats();
      populateSizes();
    }

    updateBasicSensorControls();
    updateColorToneControls();
    updateVisualAdjustmentsControls();
    Base::onupdatecontrols();

    url_ctl->setEnabled(cameraState == QImagingCamera::State_disconnected);
    cameras_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    streams_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    formats_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    sizes_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    cameraDeviceBuffers_ctl->setEnabled(cameraState == QImagingCamera::State_connected);

    setEnabled(true);
  }
}


QCheckBox * QLCSCTPCameraControls::createCheckBoxCameraControl(QSettingsWidget * sw,
    const QString & ctlid, const QString & name, const QString & tooltip)
{
  QCheckBox * ctrl =
      sw->add_checkbox(name, tooltip,
          [this, ctlid](bool checked) {
            if ( _camera ) {
              auto * c = _camera->getControl(ctlid);
              if ( c ) {
                c->value = QString::fromStdString(toString(checked));
                _camera->applyDeviceControl(*c);
              }
            }
          },
          [this, ctlid](bool * checked) {
            if ( _camera ) {
              const auto * c = _camera->getControl(ctlid);
              return c && fromString(c->value, checked);
            }
            return false;
          });

  ctrl->setObjectName(ctlid);

  return ctrl;
}

void QLCSCTPCameraControls::updateCameraControl(QCheckBox * cb, const QLCSCTPCamera::QLCCamera * cam)
{
  if (cb  && cam ) {
    const QString ctlid = cb->objectName();
    if ( !ctlid.isEmpty() ) {
      cb->setEnabled(cam->getControl(ctlid));
      return;
    }
  }
  cb->setEnabled(false);
}


QSpinBox * QLCSCTPCameraControls::createSpinBoxCameraControl(QSettingsWidget * sw, const QString & ctlid,
    const QString & name, const QString & tooltip)
{
  QSpinBox * ctrl =
      sw->add_spinbox(name, tooltip,
          [this, ctlid](int v) {
            if ( _camera ) {
              auto * c = _camera->getControl(ctlid);
              if ( c ) {
                c->value = QString::fromStdString(toString(v));
                _camera->applyDeviceControl(*c);
              }
            }
          },
          [this, ctlid](int * v) {
            if ( _camera ) {
              const auto * c = _camera->getControl(ctlid);
              return c && fromString(c->value, v);
            }
            return false;
          });

  ctrl->setObjectName(ctlid);

  return ctrl;
}

void QLCSCTPCameraControls::updateCameraControl(QSpinBox * sb, const QLCSCTPCamera::QLCCamera * cam)
{
  if (sb  && cam ) {
    const QString ctlid = sb->objectName();
    if ( !ctlid.isEmpty() ) {
      const auto * c = cam->getControl(ctlid);
      if ( c ) {
        int minv = 0, maxv = 1e6;
        fromString(c->minval, &minv);
        fromString(c->maxval, &maxv);
        sb->setRange(minv, maxv);
        sb->setEnabled(true);
        return;
      }
    }
  }
  sb->setEnabled(false);
}


QDoubleSpinBox* QLCSCTPCameraControls::createDoubleSpinBoxCameraControl(QSettingsWidget * sw, const QString & ctlid,
    const QString & name, const QString & tooltip)
{
  QDoubleSpinBox * ctrl =
      sw->add_double_spinbox(name, tooltip,
          [this, ctlid](double v) {
            if ( _camera ) {
              auto * c = _camera->getControl(ctlid);
              if ( c ) {
                c->value = QString::fromStdString(toString(v));
                _camera->applyDeviceControl(*c);
              }
            }
          },
          [this, ctlid](double * v) {
            if ( _camera ) {
              const auto * c = _camera->getControl(ctlid);
              return c && fromString(c->value, v);
            }
            return false;
          });

  ctrl->setObjectName(ctlid);

  return ctrl;
}

void QLCSCTPCameraControls::updateCameraControl(QDoubleSpinBox * sb, const QLCSCTPCamera::QLCCamera * cam)
{
  if (sb  && cam ) {
    const QString ctlid = sb->objectName();
    if ( !ctlid.isEmpty() ) {
      const auto * c = cam->getControl(ctlid);
      if ( c ) {
        double minv = 0, maxv = 1e6;
        fromString(c->minval, &minv);
        fromString(c->maxval, &maxv);
        sb->setRange(minv, maxv);
        sb->setEnabled(true);
        return;
      }
    }
  }
  sb->setEnabled(false);
}


QComboBox * QLCSCTPCameraControls::createComboBoxCameraControl(QSettingsWidget * sw, const QString & ctlid, const QString & name, const QString & tooltip)
{
  QComboBox * ctrl =
      sw->add_combobox<QComboBox>(name, tooltip, false,
          [this, ctlid](int cursel, QComboBox * combo) {
            if ( cursel >= 0 && _camera ) {
              auto * c = _camera->getControl(ctlid);
              if ( c ) {
                c->value = combo->itemText(cursel);
                _camera->applyDeviceControl(*c);
              }
            }
          },
          [this, ctlid](int * cursel, QComboBox * combo) {
            if ( _camera ) {
              const auto * c = _camera->getControl(ctlid);
              if ( c ) {
                * cursel = combo->findText(c->value);
                return true;
              }
            }
            return false;
          });

  ctrl->setObjectName(ctlid);

  return ctrl;
}

void QLCSCTPCameraControls::updateCameraControl(QComboBox * cb, const QLCSCTPCamera::QLCCamera * cam)
{
  if (cb  && cam ) {
    const QString ctlid = cb->objectName();
    if ( !ctlid.isEmpty() ) {
      const auto * c = cam->getControl(ctlid);
      if ( c ) {
        QSignalBlocker block(cb);
        const QString currentText = cb->currentText();
        cb->clear();
        for ( const std::string & s : c->values ) {
          cb->addItem(QString::fromStdString(s));
        }
        cb->setCurrentIndex(cb->findText(currentText));
        cb->setEnabled(true);
        return;
      }
    }
  }
  cb->setEnabled(false);
}

void QLCSCTPCameraControls::updateBasicSensorControls()
{
  const QLCSCTPCamera::QLCCamera * cam = _camera ? _camera->selectedCamera() : nullptr;
  updateCameraControl(AeEnable_ctl, cam);
  updateCameraControl(ExposureTime_ctl, cam);
  updateCameraControl(AnalogueGain_ctl, cam);
  updateCameraControl(AeExposureMode_ctl, cam);
  updateCameraControl(AeMeteringMode_ctl, cam);
  updateCameraControl(AeConstraintMode_ctl, cam);
  updateCameraControl(AeFlickerMode_ctl, cam);
  updateCameraControl(AeFlickerPeriod_ctl, cam);

  //updateCameraControl(LensPosition_ctl, cam);
}

void QLCSCTPCameraControls::updateColorToneControls()
{
  const QLCSCTPCamera::QLCCamera * cam = _camera ? _camera->selectedCamera() : nullptr;
  updateCameraControl(AwbEnable_ctl, cam);
  updateCameraControl(AwbMode_ctl, cam);
  updateCameraControl(ColourTemperature_ctl, cam);
}

void QLCSCTPCameraControls::updateVisualAdjustmentsControls()
{
  const QLCSCTPCamera::QLCCamera * cam = _camera ? _camera->selectedCamera() : nullptr;
  updateCameraControl(Brightness_ctl, cam);
  updateCameraControl(Contrast_ctl, cam);
  updateCameraControl(Saturation_ctl, cam);
  updateCameraControl(Gamma_ctl, cam);
  updateCameraControl(Sharpness_ctl, cam);
  updateCameraControl(NoiseReductionMode_ctl, cam);
  updateCameraControl(HdrMode_ctl, cam);
}

void QLCSCTPCameraControls::populateCameraControls()
{
  updateBasicSensorControls();
  updateColorToneControls();
  updateVisualAdjustmentsControls();
}

} /* namespace serimager */
