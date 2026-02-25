/*
 * QStereoMatcherOptions.h
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoMatcherOptions_h__
#define __QStereoMatcherOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>

#include "QStereoBMOptions.h"
#include "QStereoSGBMOptions.h"
#include "QScaleSweepOptions.h"
#ifdef HAVE_OpenCV_stereo
# include "QQuasiDenseStereoOptions.h"
# include "QStereoBinarySGBMOptions.h"
# include "QStereoBinaryBMOptions.h"
#endif // HAVE_OpenCV_stereo


class QStereoMatcherOptions :
    public QSettingsWidgetTemplate<c_regular_stereo_matcher>
{
public:
  typedef QStereoMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_regular_stereo_matcher> Base;

  QStereoMatcherOptions(QWidget * parent = nullptr);

  QEnumComboBox<stereo_matcher_type> * matcherTypeControl() const;

protected:
  QCheckBox * enabled_ctl = nullptr;
  QEnumComboBox<stereo_matcher_type> * matcher_type_ctl = nullptr;

  QStereoBMOptions * stereoBMOptions_ctl = nullptr;
  QStereoSGBMOptions * stereoSGBMOptions_ctl = nullptr;
  QScaleSweepOptions * scaleSweepOptions_ctl = nullptr;

#if HAVE_OpenCV_stereo
  QQuasiDenseStereoOptions * quasiDenseStereoOptions_ctl = nullptr;
  QStereoBinarySGBMOptions * stereoBinarySGBMOptions_ctl = nullptr;
  QStereoBinaryBMOptions * stereoBinaryBMOptions_ctl = nullptr;
#endif //HAVE_OpenCV_stereo
};

#endif /* __QStereoMatcherOptions_h__ */
