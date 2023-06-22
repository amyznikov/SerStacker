/*
 * QStereoMatcherProcessingOptions.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherProcessingOptions_h__
#define __QStereoMatcherProcessingOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_stereo_matcher_pipeline.h>

class QStereoMatcherProcessingOptions :
    public QSettingsWidget
{
public:
  typedef QStereoMatcherProcessingOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoMatcherProcessingOptions(QWidget * parent = nullptr);

  void set_processing_options(c_stereo_matcher_processing_options * options);
  c_stereo_matcher_processing_options * processing_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_matcher_processing_options * options_ = nullptr;
  QNumericBox * camera_focus_ctl = nullptr;
  QNumericBox * stereo_baseline_ctl = nullptr;
};

#endif /* __QStereoMatcherProcessingOptions_h__ */
