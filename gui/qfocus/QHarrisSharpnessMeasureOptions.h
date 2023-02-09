/*
 * QHarrisSharpnessMeasureOptions.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QHarrisSharpnessMeasureOptions_h__
#define __QHarrisSharpnessMeasureOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/focus.h>

class QHarrisSharpnessMeasureOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QHarrisSharpnessMeasureOptions ThisClass;
  typedef QSettingsWidget Base;

  QHarrisSharpnessMeasureOptions(QWidget * parent = nullptr);
  QHarrisSharpnessMeasureOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_measure_options(c_harris_sharpness_measure * options);
  const c_harris_sharpness_measure * measure_options() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  c_harris_sharpness_measure * options_ = nullptr;
  QCheckBox * avgchannel_ctl = nullptr;
};

#endif /* __QHarrisSharpnessMeasureOptions_h__ */
