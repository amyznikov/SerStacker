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
    public QSettingsWidget
{
public:
  typedef QStereoRectificationOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoRectificationOptions(QWidget * parent = nullptr);

  void set_rectification_options(c_stereo_rectification_options * options);
  c_stereo_rectification_options * rectification_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_rectification_options * options_ = nullptr;
  QCheckBox * enable_stereo_rectification_ctl = nullptr;
  QBrowsePathCombo * camera_intrinsics_yml_ctl = nullptr;
  QBrowsePathCombo * camera_extrinsics_yml_ctl = nullptr;
};

#endif /* __QStereoRectificationOptions_h__ */
