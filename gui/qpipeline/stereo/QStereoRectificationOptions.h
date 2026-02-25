/*
 * QStereoRectificationOptions.h
 *
 *  Created on: Jul 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoRectificationOptions_h__
#define __QStereoRectificationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/stereo/c_stereo_rectification_options.h>

class QStereoRectificationOptions :
    public QSettingsWidgetTemplate<c_stereo_rectification_options>
{
public:
  typedef QStereoRectificationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_stereo_rectification_options> Base;

  QStereoRectificationOptions(QWidget * parent = nullptr);

protected:
  QCheckBox * enable_stereo_rectification_ctl = nullptr;
  QBrowsePathCombo * camera_intrinsics_yml_ctl = nullptr;
  QBrowsePathCombo * camera_extrinsics_yml_ctl = nullptr;
};

#endif /* __QStereoRectificationOptions_h__ */
