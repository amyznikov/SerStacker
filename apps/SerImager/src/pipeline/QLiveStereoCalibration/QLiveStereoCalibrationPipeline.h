/*
 * QLiveStereoCalibrationPipeline.h
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationLivePipeline_h__
#define __QStereoCalibrationLivePipeline_h__

#include "QLivePipeline.h"
#include <core/pipeline/stereo_calibration/c_stereo_calibration.h>

namespace serimager {

class QLiveStereoCalibrationPipeline:
    public QLivePipeline
{
public:
  typedef QLiveStereoCalibrationPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveStereoCalibrationPipeline(const QString & name, QObject * parent = nullptr);

  c_stereo_calibration & stereo_calibration();
  const c_stereo_calibration & stereo_calibration() const;

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:

protected:
  COLORID displayColorid_ = COLORID_UNKNOWN;
  c_stereo_calibration stereo_calibration_;
  std::string output_path_;
};

} /* namespace serimager */

#endif /* __QStereoCalibrationLivePipeline_h__ */
