/*
 * QRegularStereoOptions.h
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRegularStereoOptions_h__
#define __QRegularStereoOptions_h__

#include "QStereoMatcherOptions.h"
#include "QRStereoImageProcessingOptions.h"
#include <core/pipeline/rstereo/c_regular_stereo.h>

class QRegularStereoOptions :
    public QSettingsWidget
{
public:
  typedef QRegularStereoOptions ThisClass;
  typedef QSettingsWidget Base;

  QRegularStereoOptions(QWidget * parent = nullptr);
  QRegularStereoOptions(c_regular_stereo * rstereo, QWidget * parent = nullptr);

  void set_rstereo(c_regular_stereo * rstereo);
  c_regular_stereo * rstereo() const;

  void updateRunTimeStateControls(bool isRunTime);

protected:
  void onupdatecontrols() override;

protected:
  c_regular_stereo * rstereo_ = nullptr;
  QCheckBox * enable_stereo_rectification_ctl = nullptr;
  QBrowsePathCombo * camera_intrinsics_yml_ctl = nullptr;
  QBrowsePathCombo * camera_extrinsics_yml_ctl = nullptr;
  QStereoMatcherOptions * stereoMatcherOptions_ctl = nullptr;
  QRStereoImageProcessingOptions * imageProcessingOptions_ctl = nullptr;

};

#endif /* __QRegularStereoOptions_h__ */
