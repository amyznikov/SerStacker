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
#include <core/pipeline/c_image_stacking_pipeline.h>

typedef QEnumComboBox<frame_accumulation_method>
  QFrameAccumulationMethodCombo;

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

signals:
  void applyFrameAccumulationOptionsToAllRequested(
      const c_frame_accumulation_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_frame_accumulation_options * options_ = nullptr;
  QFrameAccumulationMethodCombo * accumulation_method_ctl = nullptr;

  QNumberEditBox * lw_ctl = nullptr;
  QNumberEditBox * k_ctl = nullptr;
  QNumberEditBox * dscale_ctl = nullptr;
  QNumberEditBox * uscale_ctl = nullptr;
  QCheckBox * avgc_ctl = nullptr;

};

#endif /* __QFrameAccumulationOptions_h__ */
