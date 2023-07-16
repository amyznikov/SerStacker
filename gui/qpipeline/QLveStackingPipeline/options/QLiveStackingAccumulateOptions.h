/*
 * QLiveStackingAccumulateOptions.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveStackingAccumulateOptions_h__
#define __QLiveStackingAccumulateOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>

class QLiveStackingAccumulateOptions :
    public QSettingsWidget
{
public:
  typedef QLiveStackingAccumulateOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveStackingAccumulateOptions(QWidget * parent = nullptr);

  void set_options(c_live_stacking_accumulation_options * options);
  c_live_stacking_accumulation_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_live_stacking_accumulation_options * options_ = nullptr;
  QEnumComboBox<live_stacking_accumulation_type> * accumulation_type_ctl = nullptr;
  QCheckBox * ignore_input_mask_ctl = nullptr;
};

#endif /* __QLiveStackingAccumulateOptions_h__ */
