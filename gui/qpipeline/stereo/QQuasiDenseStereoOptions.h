/*
 * QQuasiDenseStereoOptions.h
 *
 *  Created on: Apr 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QQuasiDenseStereoOptions_h__
#define __QQuasiDenseStereoOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

#if HAVE_OpenCV_stereo

class QQuasiDenseStereoOptions :
    public QSettingsWidgetTemplate<cv::stereo::PropagationParameters>
{
public:
  typedef QQuasiDenseStereoOptions ThisClass;
  typedef QSettingsWidgetTemplate<cv::stereo::PropagationParameters> Base;

  QQuasiDenseStereoOptions(QWidget * parent = nullptr);

protected:
  QNumericBox *corrWinSize_ctl = nullptr;     // similarity window
  QNumericBox *border_ctl = nullptr;         // border to ignore

  //matching
  QNumericBox *correlationThreshold_ctl = nullptr;  // correlation threshold
  QNumericBox *textrureThreshold_ctl = nullptr;   // texture threshold

  QNumericBox *neighborhoodSize_ctl = nullptr;   // neighborhood size
  QNumericBox *disparityGradient_ctl = nullptr;  // disparity gradient threshold

  // Parameters for LK flow algorithm
  QNumericBox *lkTemplateSize_ctl = nullptr;
  QNumericBox *lkPyrLvl_ctl = nullptr;
  QNumericBox *lkTermParam1_ctl = nullptr;
  QNumericBox *lkTermParam2_ctl = nullptr;

  // Parameters for GFT algorithm.
  QNumericBox *gftQualityThres_ctl = nullptr;
  QNumericBox *gftMinSeperationDist_ctl = nullptr;
  QNumericBox *gftMaxNumFeatures_ctl = nullptr;
};

#endif // HAVE_OpenCV_stereo

#endif /* __QQuasiDenseStereoOptions_h__ */
