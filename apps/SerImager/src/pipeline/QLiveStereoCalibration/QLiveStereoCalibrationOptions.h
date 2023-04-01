/*
 * QLiveStereoCalibrationOptions.h
 *
 *  Created on: Mar 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveStereoCalibrationOptions_h__
#define __QLiveStereoCalibrationOptions_h__

#include "QLiveStereoCalibrationPipeline.h"
#include <gui/qstereocalibration/QStereoCalibrationOptions.h>

namespace serimager {

class QLiveStereoCalibrationOptions :
    public QSettingsWidget
{
public:
  typedef QLiveStereoCalibrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveStereoCalibrationOptions(QWidget * parent = nullptr);
  QLiveStereoCalibrationOptions(QLiveStereoCalibrationPipeline * pipeline_,
      QWidget * parent = nullptr);

  void setPipeline(QLiveStereoCalibrationPipeline * pipeline);
  QLiveStereoCalibrationPipeline * pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  QLiveStereoCalibrationPipeline * pipeline_ = nullptr;
  QStereoCalibrationOptions * stereoCalibrationOptions_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QLiveStereoCalibrationOptions_h__ */
