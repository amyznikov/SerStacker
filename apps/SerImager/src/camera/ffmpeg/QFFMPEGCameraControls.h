/*
 * QFFMPEGCameraControls.h
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFFMPEGCameraControls_h__
#define __QFFMPEGCameraControls_h__

#include <gui/widgets/UpdateControls.h>
#include "QCameraControlsWidget.h"
#include "QFFMPEGCameraUrlWidget.h"
#include "QFFMPEGCamera.h"

namespace serimager {

class QFFMPEGCameraControls :
    public QCameraControlsWidget
{
  Q_OBJECT;
public:
  typedef QFFMPEGCameraControls ThisClass;
  typedef QCameraControlsWidget Base;

  QFFMPEGCameraControls(const QFFMPEGCamera::sptr & camera, QWidget * parent = nullptr);
  ~QFFMPEGCameraControls();

protected :
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QFFMPEGCamera::sptr camera_;

  QFFMPEGCameraUrlWidget * url_ctl = nullptr;
  QLineEditBox * options_ctl = nullptr;
};


} /* namespace serimager */

#endif /* __QFFMPEGCameraControls_h__ */
