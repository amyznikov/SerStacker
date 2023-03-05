/*
 * QRStereoCalibrationOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoCalibrationOptions_h__
#define __QRStereoCalibrationOptions_h__

#include "QRStereoCalibrationInputOptions.h"
#include "QRStereoFeature2dOptions.h"
#include "QRStereoCalibrateOptions.h"
#include "QRStereoCalibrationOutputOptions.h"


class QRStereoCalibrationOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoCalibrationOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline);
  const c_rstereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_rstereo_calibration_pipeline::sptr pipeline_;
  QRStereoCalibrationInputOptions * inputOptions_ctl = nullptr;
  QRStereoFeature2dOptions * registration_options_ctl = nullptr;
  QRStereoCalibrateOptions * stereoCalibrateOptions_ctl = nullptr;
  QRStereoCalibrationOutputOptions * outputOptions_ctl = nullptr;
};

#endif /* __QRStereoCalibrationOptions_h__ */
