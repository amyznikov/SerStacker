/*
 * QVirtualStereoPipeline.h
 *
 *  Created on: Jul 30, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QVirtualStereoPipeline_h__
#define __QVirtualStereoPipeline_h__

#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_virtual_stereo_pipeline/c_virtual_stereo_pipeline.h>


class QVirtualStereoPipeline :
    public QImageProcessingPipelineTemplate<c_virtual_stereo_pipeline>
{
public:
  typedef QVirtualStereoPipeline ThisClass;
  typedef QImageProcessingPipelineTemplate<c_virtual_stereo_pipeline> Base;

  QVirtualStereoPipeline(const QString & name, QObject * parent = nullptr) :
    ThisClass(name, nullptr, parent)
  {
  }

  QVirtualStereoPipeline(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
    Base(name, input_sequence, parent)
  {
  }

  QPipelineSettingsWidget* createSettingsWidget(QWidget * parent = nullptr) const
  {
    return new QPipelineSettingsWidgetTemplate<ThisClass>(parent);
  }
};

#endif /* __QVirtualStereoPipeline_h__ */
