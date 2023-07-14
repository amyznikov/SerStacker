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
#if 0

#include <gui/qstereocalibration/QStereoCalibrationOptions.h>

namespace serimager {

class QLiveStereoCalibrationOptions :
    public QLivePipelineSettings<QLiveStereoCalibrationPipeline>
{
public:
  typedef QLiveStereoCalibrationOptions ThisClass;
  typedef QLivePipelineSettings<QLiveStereoCalibrationPipeline> Base;

  QLiveStereoCalibrationOptions(QWidget * parent = nullptr);
  QLiveStereoCalibrationOptions(QLiveStereoCalibrationPipeline * pipeline, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected:
  QStereoCalibrationOptions * stereoCalibrationOptions_ctl = nullptr;
};

} /* namespace serimager */

#endif // 0

#endif /* __QLiveStereoCalibrationOptions_h__ */
