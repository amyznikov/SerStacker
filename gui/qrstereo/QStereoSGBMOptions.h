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
    public QSettingsWidget
{
public:
  typedef QStereoSGBMOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoSGBMOptions(QWidget * parent = nullptr);

  void set_options(c_cvStereoSGBM_options * options);
  c_cvStereoSGBM_options* options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_cvStereoSGBM_options *options_ = nullptr;

  QNumberEditBox *minDisparity_ctl = nullptr;
  QNumberEditBox *numDisparities_ctl = nullptr;
  QNumberEditBox *blockSize_ctl = nullptr;

  QNumberEditBox *speckleWindowSize_ctl = nullptr;
  QNumberEditBox *speckleRange_ctl = nullptr;
  QNumberEditBox *disp12MaxDiff_ctl = nullptr;

  QNumberEditBox *P1_ctl = nullptr;
  QNumberEditBox *P2_ctl = nullptr;
  QNumberEditBox *preFilterCap_ctl = nullptr;
  QNumberEditBox *uniquenessRatio_ctl = nullptr;

  QEnumComboBox<StereoSGBM_Mode> *mode_ctl = nullptr;
};

#endif /* __QStereoSGBMOptions_h__ */
