/*
 * QStereoCalibrationPipeline.h
 *
 *  Created on: Jul 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationPipeline_h__
#define __QStereoCalibrationPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_stereo_calibration_pipeline/c_stereo_calibration_pipeline.h>

class QStereoCalibrationPipeline :
    public QImageProcessingPipelineTemplate<c_stereo_calibration_pipeline>
{
public:
  typedef QStereoCalibrationPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_stereo_calibration_pipeline> Base;

  QStereoCalibrationPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QStereoCalibrationPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};

#endif /* __QStereoCalibrationPipeline_h__ */
