/*
 * QRStereoCalibrationOutputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoCalibrationOutputOptions_h__
#define __QRStereoCalibrationOutputOptions_h__


#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_rstereo_calibration_pipeline.h>

class QRStereoCalibrationOutputOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoCalibrationOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoCalibrationOutputOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline);
  const c_rstereo_calibration_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_rstereo_calibration_pipeline::sptr pipeline_;
  QBrowsePathCombo * output_directory_ctl = nullptr;
  QCheckBox * save_rectified_images_ctl = nullptr;
  QLineEditBox * rectified_images_file_name_ctl = nullptr;
};

#endif /* __QRStereoCalibrationOutputOptions_h__ */
