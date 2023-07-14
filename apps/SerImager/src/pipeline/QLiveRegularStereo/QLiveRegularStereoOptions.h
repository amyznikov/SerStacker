/*
 * QLiveRegularStereoOptions.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveRegularStereoOptions_h__
#define __QLiveRegularStereoOptions_h__

#include "QLiveRegularStereoPipeline.h"
#if 0
#include <gui/qrstereo/QRegularStereoOptions.h>


namespace serimager {

class QLiveRegularStereoOptions :
    public QLivePipelineSettings<QLiveRegularStereoPipeline>
{
  Q_OBJECT;
public:
  typedef QLiveRegularStereoOptions ThisClass;
  typedef QLivePipelineSettings<QLiveRegularStereoPipeline> Base;

  QLiveRegularStereoOptions(QWidget * parent = nullptr);
  QLiveRegularStereoOptions(QLiveRegularStereoPipeline * pipeline_, QWidget * parent = nullptr);

protected:
  void update_pipeline_controls() override;

protected Q_SLOTS:
  void onLivePipelineStateChanged(bool isRunning);

protected:
  QRegularStereoOptions * regularStereoOptions_ctl = nullptr;
};

} /* namespace serimager */

#endif
#endif /* __QLiveRegularStereoOptions_h__ */
