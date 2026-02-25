/*
 * QFFMPEGCameraControls.cc
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#include "QFFMPEGCameraControls.h"
#include "QFFStreams.h"

namespace serimager {


QFFMPEGCameraControls::QFFMPEGCameraControls(const QFFMPEGCamera::sptr & camera, QWidget * parent) :
    Base(parent),
    _camera(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  url_ctl =
      add_widget<QFFMPEGCameraUrlWidget>("URL:");

  connect(url_ctl, &QFFMPEGCameraUrlWidget::urlChanged,
      [this]() {
        if ( _camera ) {
          _camera->setUrl(url_ctl->url());
          QFFStreams::save();
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _camera ) {
          url_ctl->setUrl(_camera->url());
        }
      });

  options_ctl =
      add_textbox("Options:",
          "",
          [this](const QString & value) {
            if ( _camera ) {
              _camera->setOpts(value);
              QFFStreams::save();
            }
          },
          [this](QString * value) {
            if ( _camera ) {
              * value = _camera->opts();
              return true;
            }
            return false;
          });


  connect(this, &ThisClass::enablecontrols,
      [this]() {
        const bool enable_controls = _camera && _camera->state() == QImagingCamera::State_disconnected;
        url_ctl->setEnabled(enable_controls);
        options_ctl->setEnabled(enable_controls);
      });


  if( _camera ) {

    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    connect(_camera.get(), &QFFMPEGCamera::parametersChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}

QFFMPEGCameraControls::~QFFMPEGCameraControls()
{
}

void QFFMPEGCameraControls::onCameraStateChanged()
{
  updateControls();
}
//
//void QFFMPEGCameraControls::onupdatecontrols()
//{
//  if( !_camera ) {
//    setEnabled(false);
//  }
//  else {
//
//    Base::onupdatecontrols();
//
//    url_ctl->setUrl(_camera->url());
//
//    const bool enable_controls =
//        _camera->state() == QImagingCamera::State_disconnected;
//
//    url_ctl->setEnabled(enable_controls);
//    options_ctl->setEnabled(enable_controls);
//
//    setEnabled(true);
//  }
//
//}

} /* namespace serimager */
