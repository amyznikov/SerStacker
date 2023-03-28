/*
 * QSharpnessNormMeasureOptions.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSharpnessNormMeasureOptions_h__
#define __QSharpnessNormMeasureOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/focus.h>

class QSharpnessNormMeasureOptions :
    public QSettingsWidget
{
public:
  typedef QSharpnessNormMeasureOptions ThisClass;
  typedef QSettingsWidget Base;

  QSharpnessNormMeasureOptions(QWidget * parent = nullptr);
  QSharpnessNormMeasureOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_measure_options(c_sharpness_norm_measure * options);
  const c_sharpness_norm_measure * measure_options() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  using NormType = c_sharpness_norm_measure::NormType;
  c_sharpness_norm_measure * options_ = nullptr;
  QEnumComboBox<NormType> * norm_type_ctl = nullptr;
  QNumericBox * sigma_ctl = nullptr;
};

#endif /* __QSharpnessNormMeasureOptions_h__ */
