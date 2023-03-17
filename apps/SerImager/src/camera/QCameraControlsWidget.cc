/*
 * QCameraControlsWidget.cc
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#include "QCameraControlsWidget.h"

#include "v4l2/QV4L2CameraControls.h"
#include "zwo_asi/QASICameraControls.h"
#include "ffmpeg/QFFMPEGCameraControls.h"

namespace serimager {

QCameraControlsWidget::QCameraControlsWidget(QWidget * parent) :
    Base("QCameraControlsWidget", parent)
{
}

QCameraControlsWidget * QCameraControlsWidget::create(const QImagingCamera::sptr & camera, QWidget * parent)
{
  if( QASICamera::sptr ASICamera = std::dynamic_pointer_cast<QASICamera>(camera) ) {
    return new QASICameraControls(ASICamera, parent);
  }

  if ( QV4L2Camera::sptr V4L2Camera = std::dynamic_pointer_cast<QV4L2Camera>(camera) ) {
    return new QV4L2CameraControls(V4L2Camera, parent);
  }

  if ( QFFMPEGCamera::sptr FFMPEGCamera = std::dynamic_pointer_cast<QFFMPEGCamera>(camera) ) {
    return new QFFMPEGCameraControls(FFMPEGCamera, parent);
  }

  return nullptr;
}

} /* namespace qserimager */
