/*
 * QFrameUpscaleOptions.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __QFrameUpscaleOptions_h__
#define __QFrameUpscaleOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QFrameUpscaleOptions:
    public QSettingsWidget
{
Q_OBJECT;
public:
  typedef QFrameUpscaleOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<frame_upscale_option> QUpscaleOptionCombo;
  typedef QEnumComboBox<frame_upscale_stage> QUpscaleStageCombo;


  QFrameUpscaleOptions(QWidget * parent = Q_NULLPTR);

  void set_upscale_options(c_frame_upscale_options * options);
  const c_frame_upscale_options * upscale_options() const;

signals:
  void applyFrameUpScaleOptionsToAllRequested(
      const c_frame_upscale_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_frame_upscale_options * options_ = Q_NULLPTR;
  QUpscaleOptionCombo * upscale_option_ctl = Q_NULLPTR;
  QUpscaleStageCombo * upscale_stage_ctl = Q_NULLPTR;
  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QFrameUpscaleOptions_h__ */
