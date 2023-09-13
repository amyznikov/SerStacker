/*
 * QRoiTrackerPipeline.h
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRoiTrackerPipeline_h__
#define __QRoiTrackerPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_roi_tracker_pipeline/c_roi_tracker_pipeline.h>

class QRoiTrackerPipeline :
    public QImageProcessingPipelineTemplate<c_roi_tracker_pipeline>
{
public:
public:
  typedef QRoiTrackerPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_roi_tracker_pipeline> Base;

  QRoiTrackerPipeline(const QString & name, QObject * parent = nullptr) :
      ThisClass(name, nullptr, parent)
  {
  }

  QRoiTrackerPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
      Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};

#endif /* __QRoiTrackerPipeline_h__ */
