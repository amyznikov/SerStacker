/*
 * QStereoSGBMOptions.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoSGBMOptions_h__
#define __QStereoSGBMOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

class QStereoSGBMOptions:
    public QSettingsWidgetTemplate<c_cvStereoSGBMOptions>
{
public:
  typedef QStereoSGBMOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_cvStereoSGBMOptions> Base;

  QStereoSGBMOptions(QWidget * parent = nullptr);

protected:
  QNumericBox *minDisparity_ctl = nullptr;
  QNumericBox *numDisparities_ctl = nullptr;
  QNumericBox *blockSize_ctl = nullptr;

  QNumericBox *speckleWindowSize_ctl = nullptr;
  QNumericBox *speckleRange_ctl = nullptr;
  QNumericBox *disp12MaxDiff_ctl = nullptr;

  QNumericBox *P1_ctl = nullptr;
  QNumericBox *P2_ctl = nullptr;
  QNumericBox *preFilterCap_ctl = nullptr;
  QNumericBox *uniquenessRatio_ctl = nullptr;

  QEnumComboBox<StereoSGBM_Mode> *mode_ctl = nullptr;
};

#endif /* __QStereoSGBMOptions_h__ */
