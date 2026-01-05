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
      populateStreams();
      populateFormats();
      populateSizes();
    }

    Base::onupdatecontrols();

    url_ctl->setEnabled(cameraState == QImagingCamera::State_disconnected);
    cameras_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    streams_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    formats_ctl->setEnabled(cameraState == QImagingCamera::State_connected);
    sizes_ctl->setEnabled(cameraState == QImagingCamera::State_connected);

    setEnabled(true);
  }
}

} /* namespace serimager */
