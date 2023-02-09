/*
 * QNormalizedVarianceMeasureOptions.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QNormalizedVarianceMeasureOptions_h__
#define __QNormalizedVarianceMeasureOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/focus.h>

class QNormalizedVarianceMeasureOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QNormalizedVarianceMeasureOptions ThisClass;
  typedef QSettingsWidget Base;

  QNormalizedVarianceMeasureOptions(QWidget * parent = nullptr);
  QNormalizedVarianceMeasureOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_measure_options(c_normalized_variance_measure * options);
  const c_normalized_variance_measure * measure_options() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  c_normalized_variance_measure * options_ = nullptr;
  QCheckBox * avgchannel_ctl = nullptr;
};

#endif /* __QNormalizedVarianceMeasureOptions_h__ */
