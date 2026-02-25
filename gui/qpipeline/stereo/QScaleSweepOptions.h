/*
 * QScaleSweepOptions.h
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QScaleSweepOptions_h__
#define __QScaleSweepOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

class QScaleSweepOptions :
    public QSettingsWidgetTemplate<c_ScaleSweep_options>
{
public:
  typedef QScaleSweepOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_ScaleSweep_options> Base;

  QScaleSweepOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * max_disparity_ctl = nullptr;
  QNumericBox * max_scale_ctl = nullptr;
  QNumericBox * texture_threshold_ctl = nullptr;
  QNumericBox * disp12maxDiff_ctl = nullptr;

  QBrowsePathCombo * debug_directory_ctl = nullptr;
  QNumericBox * debug_points_ctl = nullptr; // std::vector<cv::Point>

};

#endif /* __QScaleSweepOptions_h__ */
