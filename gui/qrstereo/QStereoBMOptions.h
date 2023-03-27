/*
 * QStereoBMOptions.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoBMOptions_h__
#define __QStereoBMOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

class QStereoBMOptions :
    public QSettingsWidget
{
public:
  typedef QStereoBMOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoBMOptions(QWidget * parent = nullptr);

  void set_options(c_cvStereoBM_options * options);
  c_cvStereoBM_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_cvStereoBM_options * options_ = nullptr;

  QNumberEditBox * minDisparity_ctl = nullptr;
  QNumberEditBox * numDisparities_ctl = nullptr;
  QNumberEditBox * blockSize = 21;

  QNumberEditBox * speckleWindowSize_ctl = nullptr;
  QNumberEditBox * speckleRange_ctl = nullptr;
  QNumberEditBox * disp12MaxDiff_ctl = nullptr;

  QNumberEditBox * preFilterType_ctl = nullptr;
  QNumberEditBox * preFilterSize_ctl = nullptr;
  QNumberEditBox * preFilterCap_ctl = nullptr;
  QNumberEditBox * textureThreshold_ctl = nullptr;
  QNumberEditBox * uniquenessRatio_ctl = nullptr;
  QNumberEditBox * smallerBlockSize_ctl = nullptr;
};

#endif /* __QStereoBMOptions_h__ */
