/*
 * QBackgroundNormalizationOptions.h
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QBackgroundNormalizationOptions_h__
#define __QBackgroundNormalizationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/white_balance/histogram_normalization.h>

class QBackgroundNormalizationOptions:
    public QSettingsWidget
{
public:
  typedef QBackgroundNormalizationOptions ThisClass;
  typedef QSettingsWidget Base;

  QBackgroundNormalizationOptions(QWidget * parent = nullptr);

  void set_options(c_histogram_normalization_options * options);
  c_histogram_normalization_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_histogram_normalization_options * options_ = nullptr;

  QEnumComboBox<histogram_normalization_type> * norm_type_ctl = nullptr;
  QNumericBox * stretch_ctl = nullptr;
  QNumericBox * offset_ctl = nullptr;
};

#endif /* __QBackgroundNormalizationOptions_h__ */
