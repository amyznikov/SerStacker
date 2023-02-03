/*
 * QTestGLViewSettings.h
 *
 *  Created on: Feb 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QTestGLViewSettings_h__
#define __QTestGLViewSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QTestGLView.h"

namespace qgltest {

class QTestGLViewSettings :
    public QSettingsWidget
{
public:
  typedef QTestGLViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QTestGLViewSettings(QWidget * parent = nullptr);

  void setGLView(QTestGLView * view);
  QTestGLView *  glView() const;

protected:
  void onupdatecontrols() override;

protected:
  QTestGLView * glView_ = nullptr;
  QDoubleSliderSpinBox * eye_x_ctl = nullptr;
  QDoubleSliderSpinBox * eye_y_ctl = nullptr;
  QDoubleSliderSpinBox * eye_z_ctl = nullptr;

};

} /* namespace qgltest */

#endif /* __QTestGLViewSettings_h__ */
