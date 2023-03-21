/*
 * QLiveStereoCalibrationPipeline.h
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationLivePipeline_h__
#define __QStereoCalibrationLivePipeline_h__

#include "../../QLivePipeline.h"

namespace serimager {

class QLiveStereoCalibrationPipeline:
    public QLivePipeline
{
public:
  typedef QLiveStereoCalibrationPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveStereoCalibrationPipeline(const QString & name, QObject * parent = nullptr);

  bool processFrame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool getDisplayImage(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

protected:
  cv::Mat displayImage_;
  COLORID displayColorid_ = COLORID_UNKNOWN;
};

} /* namespace serimager */

#endif /* __QStereoCalibrationLivePipeline_h__ */
