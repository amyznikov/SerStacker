/*
 * QStereoMatcherPipelineOptions.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherPipelineOptions_h__
#define __QStereoMatcherPipelineOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qstereoinput/QStereoInputOptions.h>
#include <gui/qrstereo/QRegularStereoOptions.h>
#include "QStereoMatcherOutputOptions.h"
#include "QStereoMatcherProcessingOptions.h"
#include <core/pipeline/c_stereo_matcher_pipeline.h>

class QStereoMatcherPipelineOptions :
    public QSettingsWidget
{
public:
  typedef QStereoMatcherPipelineOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoMatcherPipelineOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_stereo_matcher_pipeline::sptr & pipeline);
  const c_stereo_matcher_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_stereo_matcher_pipeline::sptr pipeline_;
  QStereoInputOptions * inputOptions_ctl = nullptr;
  QRegularStereoOptions * regularStereoOptions_ctl = nullptr;
  QStereoMatcherProcessingOptions * processingOptions_ctl = nullptr;
  QStereoMatcherOutputOptions * outputOptions_ctl = nullptr;

};

#endif /* __QStereoMatcherPipelineOptions_h__ */
