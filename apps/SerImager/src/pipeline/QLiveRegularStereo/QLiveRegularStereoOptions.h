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
#include <gui/qrstereo/QRegularStereoOptions.h>


namespace serimager {

class QLiveRegularStereoOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLiveRegularStereoOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveRegularStereoOptions(QWidget * parent = nullptr);
  QLiveRegularStereoOptions(QLiveRegularStereoPipeline * pipeline_, QWidget * parent = nullptr);

  void setPipeline(QLiveRegularStereoPipeline * pipeline);
  QLiveRegularStereoPipeline * pipeline() const;

protected Q_SLOTS:
  void onLivePipelineStateChanged(bool isRunning);

protected:
  void onupdatecontrols() override;

protected:
  QLiveRegularStereoPipeline * pipeline_ = nullptr;
  QRegularStereoOptions * regularStereoOptions_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QLiveRegularStereoOptions_h__ */
