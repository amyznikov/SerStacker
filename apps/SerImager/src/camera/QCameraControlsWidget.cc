/*
 * QCameraControlsWidget.cc
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#include "QCameraControlsWidget.h"

#include "v4l2/QV4L2CameraControls.h"
#include "zwo_asi/QASICameraControls.h"

namespace serimager {

QCameraControlsWidget::QCameraControlsWidget(QWidget * parent) :
    Base("QCameraControlsWidget", parent)
{
}

QCameraControlsWidget * QCameraControlsWidget::create(const QImagingCamera::sptr & camera, QWidget * parent)
{
  QASICamera::sptr ASICamera =
      std::dynamic_pointer_cast<QASICamera>(camera);
  if ( ASICamera ) {
    return new QASICameraControls(ASICamera, parent);
  }


  QV4L2Camera::sptr V4L2Camera =
      std::dynamic_pointer_cast<QV4L2Camera>(camera);
  if ( V4L2Camera ) {
    return new QV4L2CameraControls(V4L2Camera, parent);
  }

  return nullptr;
}

} /* namespace qserimager */
