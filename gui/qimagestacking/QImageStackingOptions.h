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
#include "QFrameAccumulationOptions.h"
#include "QFrameRegistrationOptions.h"
#include "QFrameUpscaleOptions.h"
#include "QImageProcessingOptions.h"
#include "QImageStackingInputOptions.h"
#include "QROISelectionOptions.h"
#include "QStackOutputOptions.h"



class QImageStackingOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageStackingOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageStackingOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_image_stacking_pipeline::sptr & pipeline);
  const c_image_stacking_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_pipeline::sptr current_pipeline_;

  //QLineEditBox * stackName_ctl = nullptr;
  QImageStackingInputOptions * inputOptions_ctl = nullptr;
  QROISelectionOptions * roiSelection_ctl = nullptr;
  QFrameUpscaleOptions * upscaleOptions_ctl = nullptr;
  QFrameRegistrationOptions * frameRegistration_ctl = nullptr;
  QFrameAccumulationOptions * frameAccumulation_ctl = nullptr;
  QImageProcessingOptions * imageProcessingOptions_ctl = nullptr;
  QStackOutputOptions * outputOptions_ctl = nullptr;
};


#endif /* __QStackingOptions_h__ */
