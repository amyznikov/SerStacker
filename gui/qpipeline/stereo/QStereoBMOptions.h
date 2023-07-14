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

  void set_options(c_cvStereoBMOptions * options);
  c_cvStereoBMOptions * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_cvStereoBMOptions * options_ = nullptr;

  QNumericBox * minDisparity_ctl = nullptr;
  QNumericBox * numDisparities_ctl = nullptr;
  QNumericBox * blockSize_ctl = nullptr;

  QNumericBox * speckleWindowSize_ctl = nullptr;
  QNumericBox * speckleRange_ctl = nullptr;
  QNumericBox * disp12MaxDiff_ctl = nullptr;

  QEnumComboBox<StereoBM_PreFilterType> * preFilterType_ctl = nullptr;
  QNumericBox * preFilterSize_ctl = nullptr;
  QNumericBox * preFilterCap_ctl = nullptr;
  QNumericBox * textureThreshold_ctl = nullptr;
  QNumericBox * uniquenessRatio_ctl = nullptr;
  QNumericBox * smallerBlockSize_ctl = nullptr;
};

#endif /* __QStereoBMOptions_h__ */
