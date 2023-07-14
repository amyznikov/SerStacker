/*
 * QROISelectionOptions.h
 *
 *  Created on: Jul 17, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QROISelectionOptions_h__
#define __QROISelectionOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>

typedef QEnumComboBox<roi_selection_method>
  QROISelectionMethodCombo;

class QROISelectionOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QROISelectionOptions ThisClass;
  typedef QSettingsWidget Base;

  QROISelectionOptions(QWidget * parent = nullptr);

  void set_roi_selection_options(c_roi_selection_options * options);
  const c_roi_selection_options * roi_selection_options() const;

Q_SIGNALS:
  void applyROISelectionOptionsToAllRequested(
      const c_roi_selection_options & options);

protected:
  void onupdatecontrols() override;
  void updateROIControls();

protected:
  c_roi_selection_options * options_ = nullptr;
  QROISelectionMethodCombo * selectionMethod_ctl = nullptr;
  QNumericBox * rectangeROI_ctl = nullptr;
  QNumericBox * planetaryDiskSize_ctl = nullptr;
  QNumericBox * planetaryDiskGbSigma_ctl = nullptr;
  QNumericBox * planetaryDiskStdevFactor_ctl = nullptr;
//  QToolButton * applyToAll_ctl = nullptr;
};

#endif /* __QROISelectionOptions_h__ */
