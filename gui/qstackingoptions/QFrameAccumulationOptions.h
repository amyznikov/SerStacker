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

QString toString(enum frame_upscale_option v);
enum frame_upscale_option fromString(const QString  & s,
    enum frame_upscale_option defval );

QString toString(enum frame_accumulation_method v);
enum frame_accumulation_method fromString(const QString  & s,
    enum frame_accumulation_method defval );

class QFrameAccumulationMethodCombo :
    public QEnumComboBox<frame_accumulation_method>
{
  Q_OBJECT;
public:
  typedef QEnumComboBox<frame_accumulation_method> Base;

  QFrameAccumulationMethodCombo(QWidget * parent = Q_NULLPTR)
      : Base(parent, frame_accumulation_methods)
    {}
};

class QFrameUpscaleOptionCombo :
    public QEnumComboBox<frame_upscale_option>
{
  Q_OBJECT;
public:
  typedef QEnumComboBox<frame_upscale_option> Base;

  QFrameUpscaleOptionCombo(QWidget * parent = Q_NULLPTR)
      : Base(parent, frame_upscale_options)
    {}
};

class QFrameAccumulationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameAccumulationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameAccumulationOptions(QWidget * parent = Q_NULLPTR);

  void set_accumulation_options(c_frame_accumulation_options * options);
  const c_frame_accumulation_options * accumulation_options() const;

signals:
  void applyFrameAccumulationOptionsToAllRequested(
      const c_frame_accumulation_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_frame_accumulation_options * options_ = Q_NULLPTR;
  QFrameAccumulationMethodCombo * accumulation_method_ctl = Q_NULLPTR;
  QFrameUpscaleOptionCombo * upscale_option_ctl = Q_NULLPTR;
  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QFrameAccumulationOptions_h__ */
