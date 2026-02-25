/*
 * QScaleSweepStereoMatcherOptions.h
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QScaleSweepStereoMatcherOptions_h__
#define __QScaleSweepStereoMatcherOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_regular_stereo_pipeline/c_regular_stereo_pipeline.h>

class QScaleSweepStereoMatcherOptions :
    public QSettingsWidget
{
public:
  typedef QScaleSweepStereoMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QScaleSweepStereoMatcherOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void populatesources();

protected:
  c_regular_stereo_pipeline::sptr _pipeline;

  QCheckBox * enable_stereo_matching_ctl = nullptr;
  QNumericBox * max_disparity_ctl = nullptr;
  QNumericBox * max_scale_ctl = nullptr;
  QNumericBox * texture_threshold_ctl = nullptr;
  QNumericBox * disp12maxDiff_ctl = nullptr;
  QCheckBox * save_debug_images_ctl = nullptr;
  QCheckBox * process_only_debug_frames_ctl = nullptr;
  QNumericBox * debug_frames_ctl = nullptr;
  QNumericBox * debug_points_ctl = nullptr;

};

#endif /* __QScaleSweepStereoMatcherOptions_h__ */
