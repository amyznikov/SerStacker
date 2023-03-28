/*
 * QFrameAccumulationOptions.h
 *
 *  Created on: Jul 13, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFrameAccumulationOptions_h__
#define __QFrameAccumulationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qfocus/QLPGSharpnessMeasureOptions.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

typedef QEnumComboBox<frame_accumulation_method>
  QFrameAccumulationMethodCombo;

class QFocusStackingOptions;

class QFrameAccumulationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameAccumulationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameAccumulationOptions(QWidget * parent = nullptr);

  void set_accumulation_options(c_frame_accumulation_options * options);
  const c_frame_accumulation_options * accumulation_options() const;

Q_SIGNALS:
  void applyFrameAccumulationOptionsToAllRequested(
      const c_frame_accumulation_options & options);

protected:
  void onupdatecontrols() override;
  void updatecurrentoptionswidget();

protected:
  c_frame_accumulation_options * options_ = nullptr;
  QFrameAccumulationMethodCombo * accumulation_method_ctl = nullptr;
  QStackedWidget * stack_ctl = nullptr;
  QLPGSharpnessMeasureOptions * lpg_options_ctl = nullptr;
  QFocusStackingOptions * focus_stack_options_ctl = nullptr;;
};


class QFocusStackingOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFocusStackingOptions ThisClass;
  typedef QSettingsWidget Base;

  QFocusStackingOptions(QWidget * parent = nullptr);

  void set_accumulation_options(c_frame_accumulation_options * options);
  const c_frame_accumulation_options * accumulation_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_frame_accumulation_options * options_ = nullptr;
  QEnumComboBox<c_laplacian_pyramid_focus_stacking::fusing_policy> * fusion_ctl = nullptr;
  QCheckBox * avgchannel_ctl = nullptr;
  QCheckBox * enable_inpaint_ctl = nullptr;
  QNumericBox * kradius_ctl = nullptr;
  QNumericBox * ksigma_ctl = nullptr;

};

#endif /* __QFrameAccumulationOptions_h__ */
