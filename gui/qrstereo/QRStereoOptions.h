/*
 * QRStereoOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoOptions_h__
#define __QRStereoOptions_h__

#include "QRStereoInputOptions.h"
#include "QRStereoFeature2dOptions.h"
#include "QRStereoCalibrateOptions.h"
#include "QRStereoImageProcessingOptions.h"
#include "QRStereoOutputOptions.h"
#include <gui/qpipeline/stereo/QScaleSweepStereoMatcherOptions.h>


class QRStereoOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_regular_stereo_pipeline::sptr pipeline_;
  QRStereoInputOptions * inputOptions_ctl = nullptr;
  QRStereoFeature2dOptions * feature2DOptions_ctl = nullptr;
  QRStereoCalibrateOptions * stereoCalibrateOptions_ctl = nullptr;
  QScaleSweepStereoMatcherOptions * stereoMatchingOptions_ctl = nullptr;
  QRStereoImageProcessingOptions * imageProcessingOptions_ctl = nullptr;
  QRStereoOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QRStereoOptions_h__ */
