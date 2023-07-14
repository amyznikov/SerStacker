/*
 * QStereoMatcherInputOptions.h
 *
 *  Created on: Jul 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherInputOptions_h__
#define __QStereoMatcherInputOptions_h__

#include <gui/qpipeline/stereo/QStereoInputOptions.h>
#include <core/pipeline/c_stereo_matcher_pipeline/c_stereo_matcher_pipeline.h>

class QStereoMatcherInputOptions :
    public QStereoInputOptions<c_stereo_matcher_pipeline>
{
public:
  typedef QStereoMatcherInputOptions ThisClass;
  typedef QStereoInputOptions<c_stereo_matcher_pipeline> Base;

  QStereoMatcherInputOptions(QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;
};

#endif /* __QStereoMatcherInputOptions_h__ */
