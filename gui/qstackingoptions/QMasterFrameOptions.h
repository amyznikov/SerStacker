/*
 * QMasterFrameOptions.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QStackingMasterFrameSettings_h__
#define __QStackingMasterFrameSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QMasterFrameOptions
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QMasterFrameOptions ThisClass;
  typedef QSettingsWidget Base;

  QMasterFrameOptions(QWidget * parent = Q_NULLPTR);

  void set_master_frame_options(c_master_frame_options * options, const c_input_sequence::ptr & available_sources);
  const c_master_frame_options * master_frame_options() const;
  const c_input_sequence::ptr & input_sequence() const;

signals:
  void applyMasterFrameSettingsToAllRequested(const c_master_frame_options & options);

protected:
  void onupdatecontrols() override;
  QString browseForMasterFrame();
  QString browseForMasterFFTSPath();
  void updateMasterSourceBasingOnComboboxItemIndex(int comboboxItemIndex);
  void updateMasterFrameIndex();

protected slots:
  void onMasterSourceComboCurrentIndexChanged(int);
  void onSpinBoxValueChanged(int value);
  void onGenerateMasterFrameCheckboxStateChanged(int);
  void onEccFlowScaleChanged();
  void onAccumulateMasterFlowCheckboxStateChanged(int);
  void onSaveMasterFrameCheckboxStateChanged(int);
  void onMaxFramesForMasterFrameGenerationChanged();
  void onApplyInputFramePprocessorCheckboxStateChanged(int);

protected:
  c_master_frame_options * options_ = Q_NULLPTR;
  c_input_sequence::ptr input_sequence_;

  QComboBox * masterSource_ctl = Q_NULLPTR;
  QSpinBox * masterFrameIndex_ctl = Q_NULLPTR;
  QCheckBox * apply_input_frame_processor_ctl = Q_NULLPTR;
  QCheckBox * generateMasterFrame_ctl = Q_NULLPTR;
  QNumberEditBox * maxFramesForMasterFrameGeneration_ctl = Q_NULLPTR;
  QNumberEditBox * eccFlowScale_ctl = Q_NULLPTR;
  QCheckBox * compensateMasterFlow_ctl = Q_NULLPTR;
  QCheckBox * saveMasterFrame_ctl = Q_NULLPTR;

  QToolButton * applyToAll_ctl = Q_NULLPTR;
  int previousComboboxItemIndex = -1;

};

#endif /* __QStackingMasterFrameSettings_h__ */
