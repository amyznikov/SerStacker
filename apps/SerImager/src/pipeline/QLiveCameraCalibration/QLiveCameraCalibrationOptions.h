/*
 * QLiveCameraCalibrationOptions.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveCameraCalibrationOptions_h__
#define __QLiveCameraCalibrationOptions_h__

#include "QLiveCameraCalibrationPipeline.h"
#if 0

#include <gui/qcameracalibration/QCameraCalibrationOptions.h>

namespace serimager {

class QLiveCameraCalibrationOptions :
    public QLivePipelineSettings<QLiveCameraCalibrationPipeline>
{
public:
  typedef QLiveCameraCalibrationOptions ThisClass;
  typedef  QLivePipelineSettings<QLiveCameraCalibrationPipeline> Base;

  QLiveCameraCalibrationOptions(QWidget * parent = nullptr);
  QLiveCameraCalibrationOptions(QLiveCameraCalibrationPipeline * pipeline, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QCameraCalibrationOptions * calibrationOptions_ctl = nullptr;
};

} /* namespace serimager */

#endif // 0

#endif /* __QLiveCameraCalibrationOptions_h__ */
