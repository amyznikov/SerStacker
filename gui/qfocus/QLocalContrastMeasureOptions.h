/*
 * QLocalContrastMeasureOptions.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLocalContrastMeasureOptions_h__
#define __QLocalContrastMeasureOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/focus.h>


class QLocalContrastMeasureOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLocalContrastMeasureOptions ThisClass;
  typedef QSettingsWidget Base;

  QLocalContrastMeasureOptions(QWidget * parent = nullptr);
  QLocalContrastMeasureOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_measure_options(c_local_contrast_measure * options);
  const c_local_contrast_measure * measure_options() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  c_local_contrast_measure * options_ = nullptr;
  QCheckBox * avgchannel_ctl = nullptr;
};

#endif /* __QLocalContrastMeasureOptions_h__ */
