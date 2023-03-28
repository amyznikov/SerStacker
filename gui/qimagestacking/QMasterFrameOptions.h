/*
 * QMasterFrameOptions.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QStackingMasterFrameSettings_h__
#define __QStackingMasterFrameSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QMasterFrameOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QMasterFrameOptions ThisClass;
  typedef QSettingsWidget Base;

  QMasterFrameOptions(QWidget * parent = nullptr);

  void set_current_pipeline(const c_image_stacking_pipeline::sptr & current_pipeline);
  const c_image_stacking_pipeline::sptr & current_pipeline() const;

protected:
  void onupdatecontrols() override;
  QString browseForMasterFrame();
  QString browseForMasterFFTSPath();
  void updateMasterSourceBasingOnComboboxItemIndex(int comboboxItemIndex);
  void updateMasterFrameIndex();

protected Q_SLOTS:
  void onMasterSourceComboCurrentIndexChanged(int);
  void onMasterFrameSeletionMethodChaned(master_frame_selection_method v);
  void onSpinBoxValueChanged(int value);
  void onGenerateMasterFrameCheckboxStateChanged(int);
  void onEccFlowScaleChanged();
  void onMasterSharpenFactorChanged();
  void onAccumulatedSharpenFactorChanged();

  void onSaveMasterFrameCheckboxStateChanged(int);
  void onMaxFramesForMasterFrameGenerationChanged();
  void onApplyInputFramePprocessorCheckboxStateChanged(int);

protected:
  c_image_stacking_pipeline::sptr current_pipeline_;
  c_master_frame_options * options_ = nullptr;

  QComboBox * masterSource_ctl = nullptr;
  QEnumComboBox<master_frame_selection_method> * masterFrameSelectionMethod_ctl = nullptr;
  QSpinBox * masterFrameIndex_ctl = nullptr;
  QCheckBox * apply_input_frame_processors_ctl = nullptr;
  QCheckBox * generateMasterFrame_ctl = nullptr;
  QNumericBox * maxFramesForMasterFrameGeneration_ctl = nullptr;
  QNumericBox * eccFlowScale_ctl = nullptr;
  QNumericBox * master_sharpen_factor_ctl = nullptr;
  QNumericBox * accumulated_sharpen_factor_ctl = nullptr;

  //QCheckBox * compensateMasterFlow_ctl = nullptr;
  QCheckBox * saveMasterFrame_ctl = nullptr;

  //QToolButton * applyToAll_ctl = nullptr;
  int previousComboboxItemIndex = -1;

};

#endif /* __QStackingMasterFrameSettings_h__ */
