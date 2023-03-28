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
    public QSettingsWidget
{
public:
  typedef QScaleSweepOptions ThisClass;
  typedef QSettingsWidget Base;

  QScaleSweepOptions(QWidget * parent = nullptr);

  void set_options(c_ScaleSweep_options * options);
  c_ScaleSweep_options* options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_ScaleSweep_options *options_ = nullptr;

  QNumericBox * max_disparity_ctl = nullptr;
  QNumericBox * max_scale_ctl = nullptr;
  QNumericBox * kernel_sigma_ctl = nullptr;
  QNumericBox * kernel_radius_ctl = nullptr;

  QBrowsePathCombo * debug_directory_ctl = nullptr;
  QNumericBox * debug_points_ctl = nullptr; // std::vector<cv::Point>

};

#endif /* __QScaleSweepOptions_h__ */
