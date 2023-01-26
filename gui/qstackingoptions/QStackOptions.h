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
#include "QROISelectionOptions.h"
#include "QFrameUpscaleOptions.h"
#include "QFrameRegistrationOptions.h"
#include "QFrameAccumulationOptions.h"
#include "QImageProcessingOptions.h"
#include "QStackOutputOptions.h"



class QStackingSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QStackingSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QStackingSettingsWidget(QWidget * parent = nullptr);

  void setCurrentStack(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & currentStack() const;

Q_SIGNALS:
  void stackOptionsChanged();
  void stackNameChanged(const c_image_stacking_options::ptr & stack);
  void applyInputOptionsToAllRequested(const c_input_options & stack);
  //void applyMasterFrameOptionsToAllRequested(const c_master_frame_options & stack);
  void applyROISelectionOptionsToAllRequested(const c_roi_selection_options & stack);
  void applyFrameUpscaleOptionsToAllRequested(const c_frame_upscale_options & stack);
  void applyFrameAccumulationOptionsToAllRequested(const c_frame_accumulation_options & stack);
  void applyFrameRegistrationOptionsToAllRequested(const c_image_stacking_options::ptr & stack);
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & stack);

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr stack_;

  QLineEditBox * stackName_ctl = nullptr;
  QImageStackingInputOptions * inputOptions_ctl = nullptr;
  QROISelectionOptions * roiSelection_ctl = nullptr;
  QFrameUpscaleOptions * upscaleOptions_ctl = nullptr;
  QFrameRegistrationOptions * frameRegistration_ctl = nullptr;
  QFrameAccumulationOptions * frameAccumulation_ctl = nullptr;
  QImageProcessingOptions * imageProcessingOptions_ctl = nullptr;
  QStackOutputOptions * outputOptions_ctl = nullptr;
};


class QStackOptions :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QStackOptions ThisClass;
  typedef QWidget Base;

  QStackOptions(QWidget * parent = nullptr);

  void setCurrentStack(const c_image_stacking_options::ptr & options);
  const c_image_stacking_options::ptr & currentStack() const;

  void updateControls();

Q_SIGNALS:
  void stackOptionsChanged();
  void stackNameChanged(const c_image_stacking_options::ptr & stack);
  void closeWindowRequested();

  void applyInputOptionsToAllRequested(const c_input_options & options);
  //void applyMasterFrameOptionsToAllRequested(const c_master_frame_options & options);
  void applyROISelectionOptionsToAllRequested(const c_roi_selection_options & options);
  void applyFrameUpscaleOptionsToAllRequested(const c_frame_upscale_options & options);
  void applyFrameAccumulationOptionsToAllRequested(const c_frame_accumulation_options & options);
  void applyFrameRegistrationOptionsToAllRequested(const c_image_stacking_options::ptr & stack);
  void applyOutputOptionsToAllRequested(const c_image_stacking_output_options & options);
  void applyAllStackOptionsToAllRequested(const c_image_stacking_options::ptr & stack);

protected:
  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QStackingSettingsWidget * stackSettings_ctl = nullptr;
  QScrollArea * scrollArea_ = nullptr;
};

#endif /* __QStackingOptions_h__ */
