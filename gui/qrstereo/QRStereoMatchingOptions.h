/*
 * QRStereoMatchingOptions.h
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRStereoMatchingOptions_h__
#define __QRStereoMatchingOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_regular_stereo_pipeline.h>

class QRStereoMatchingOptions :
    public QSettingsWidget
{
public:
  typedef QRStereoMatchingOptions ThisClass;
  typedef QSettingsWidget Base;

  QRStereoMatchingOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline);
  const c_regular_stereo_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;
  void populatesources();

protected:
  c_regular_stereo_pipeline::sptr pipeline_;

  QCheckBox * enable_stereo_matching_ctl = nullptr;
  QNumberEditBox * max_disparity_ctl = nullptr;
  QNumberEditBox * max_scale_ctl = nullptr;

};

#endif /* __QRStereoMatchingOptions_h__ */
