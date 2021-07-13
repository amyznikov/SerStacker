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
#include "QFrameRegistrationOptions.h"
#include "QMasterFrameOptions.h"
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
  void applyOutputSettingsToAllRequested(const c_image_stacking_output_options & options);
  void applyMasterFrameSettingsToAllRequested(const c_stacking_master_frame_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr stack_;

  QLineEditBox * stackName_ctl = Q_NULLPTR;
  QMasterFrameOptions * masterFrameSettings_ctl = Q_NULLPTR;
  QFrameAccumulationSettings * frameAccumulationSettings_ctl = Q_NULLPTR;
  QFrameRegistrationOptions * frameRegistrationSettings_ctl = Q_NULLPTR;
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
  void applyRegistrationSettingsToAllRequested(const c_image_stacking_options::ptr & currentStack);
  void applyOutputSettingsToAllRequested(const c_image_stacking_output_options & options);
  void applyMasterFrameSettingsToAllRequested(const c_stacking_master_frame_options & options);

protected:
  QVBoxLayout * layout_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStackingSettingsWidget * stackSettings_ctl = Q_NULLPTR;
  QScrollArea * scrollArea_ = Q_NULLPTR;
};

#endif /* __QStackingOptions_h__ */
