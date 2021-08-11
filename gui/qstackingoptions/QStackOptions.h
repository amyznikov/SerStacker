/*
 * QStackingOptions.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QStackingOptions_h__
#define __QStackingOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QLineEditBox.h>
#include "QImageStackingInputOptions.h"
#include "QMasterFrameOptions.h"
#include "QROISelectionOptions.h"
#include "QFrameUpscaleOptions.h"
#include "QFrameRegistrationOptions.h"
#include "QFrameAccumulationOptions.h"
#include "QStackOutputOptions.h"



class QStackingSettingsWidget
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QStackingSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QStackingSettingsWidget(QWidget * parent = Q_NULLPTR);

  void setCurrentStack(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & currentStack() const;

signals:
  void stackNameChanged(const c_image_stacking_options::ptr & pipeline);
  void applyInputOptionsToAllRequested(const c_input_options & options);
  void applyMasterFrameOptionsToAllRequested(const c_master_frame_options & options);
  void applyROISelectionOptionsToAllRequested(const c_roi_selection_options & options);
  void applyFrameUpscaleOptionsToAllRequested(const c_frame_upscale_options & options);
  void applyFrameAccumulationOptionsToAllRequested(const c_frame_accumulation_options & options);
  void applyFrameRegistrationOptionsToAllRequested(const c_frame_registration_options & options);
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr stack_;

  QLineEditBox * stackName_ctl = Q_NULLPTR;
  QImageStackingInputOptions * inputOptions_ctl = Q_NULLPTR;
  QMasterFrameOptions * masterFrame_ctl = Q_NULLPTR;
  QROISelectionOptions * roiSelection_ctl = Q_NULLPTR;
  QFrameUpscaleOptions * upscaleOptions_ctl = Q_NULLPTR;
  QFrameAccumulationOptions * frameAccumulation_ctl = Q_NULLPTR;
  QFrameRegistrationOptions * frameRegistration_ctl = Q_NULLPTR;
  QStackOutputOptions * outputOptions_ctl = Q_NULLPTR;
};


class QStackOptions
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QStackOptions ThisClass;
  typedef QWidget Base;

  QStackOptions(QWidget * parent = Q_NULLPTR);

  void setCurrentStack(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & currentStack() const;

  void updateControls();

signals:
  void stackNameChanged(const c_image_stacking_options::ptr & stack);
  void closeWindowRequested();

  void applyInputOptionsToAllRequested(const c_input_options & options);
  void applyMasterFrameOptionsToAllRequested(const c_master_frame_options & options);
  void applyROISelectionOptionsToAllRequested(const c_roi_selection_options & options);
  void applyFrameUpscaleOptionsToAllRequested(const c_frame_upscale_options & options);
  void applyFrameAccumulationOptionsToAllRequested(const c_frame_accumulation_options & options);
  void applyFrameRegistrationOptionsToAllRequested(const c_frame_registration_options & options);
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);
  void applyAllStackOptionsToAllRequested(const c_image_stacking_options::ptr & currentStack);

protected:
  QVBoxLayout * layout_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStackingSettingsWidget * stackSettings_ctl = Q_NULLPTR;
  QScrollArea * scrollArea_ = Q_NULLPTR;
};

#endif /* __QStackingOptions_h__ */
