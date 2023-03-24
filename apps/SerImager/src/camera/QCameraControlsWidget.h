/*
 * QCameraControlsWidget.h
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraSettingsWidget_h__
#define __QCameraSettingsWidget_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QImagingCamera.h"

namespace serimager {

class QCameraControlsWidget:
    public QSettingsWidget
{
public:
  typedef QCameraControlsWidget ThisClass;
  typedef QSettingsWidget Base;

  static QCameraControlsWidget * create(const QImagingCamera::sptr & camera,
      QWidget * parent = nullptr);

protected:
  QCameraControlsWidget(QWidget * parent = nullptr);
};

} /* namespace serimager */

#endif /* __QCameraSettingsWidget_h__ */
