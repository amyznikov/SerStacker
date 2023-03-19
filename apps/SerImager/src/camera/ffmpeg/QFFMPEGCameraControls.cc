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
    camera_(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  url_ctl =
      add_widget<QFFMPEGCameraUrlWidget>("URL:");

  connect(url_ctl, &QFFMPEGCameraUrlWidget::urlChanged,
      [this]() {
        if ( camera_ ) {
          camera_->setUrl(url_ctl->url());
          QFFStreams::save();
        }
      });

  options_ctl =
      add_textbox("Options:",
          [this](const QString & value) {
            if ( camera_ ) {
              camera_->setOpts(value);
              QFFStreams::save();
            }
          },
          [this](QString * value) {
            if ( camera_ ) {
              * value = camera_->opts();
              return true;
            }
            return false;
          });


  if( camera_ ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    connect(camera_.get(), &QFFMPEGCamera::parametersChanged,
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

void QFFMPEGCameraControls::onupdatecontrols()
{
  if( !camera_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    url_ctl->setUrl(camera_->url());

    const bool enable_controls =
        camera_->state() == QImagingCamera::State_disconnected;

    url_ctl->setEnabled(enable_controls);
    options_ctl->setEnabled(enable_controls);

    setEnabled(true);
  }

}

} /* namespace serimager */
