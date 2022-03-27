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
#include <core/pipeline/c_image_stacking_pipeline.h>

//
//QString toString(enum roi_selection_method v);
//enum roi_selection_method fromString(const QString  & s,
//    enum roi_selection_method defval );

typedef QEnumComboBox<roi_selection_method> QROISelectionMethodCombo;

//class QROISelectionMethodCombo :
//    public QEnumComboBox<roi_selection_method>
//{
//  Q_OBJECT;
//public:
//  typedef QEnumComboBox<roi_selection_method> Base;
//
//  QROISelectionMethodCombo(QWidget * parent = Q_NULLPTR)
//      : Base(parent)
//    {}
//};
//

class QROISelectionOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QROISelectionOptions ThisClass;
  typedef QSettingsWidget Base;

  QROISelectionOptions(QWidget * parent = Q_NULLPTR);

  void set_roi_selection_options(c_roi_selection_options * options);
  const c_roi_selection_options * roi_selection_options() const;

signals:
  void applyROISelectionOptionsToAllRequested(
      const c_roi_selection_options & options);

protected:
  void onupdatecontrols() override;
  void updateROIControls();

protected:
  c_roi_selection_options * options_ = Q_NULLPTR;
  QROISelectionMethodCombo * selectionMethod_ctl = Q_NULLPTR;
  QNumberEditBox * planetaryDiskSize_ctl = Q_NULLPTR;
  QNumberEditBox * rectangeROI_ctl = Q_NULLPTR;
  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QROISelectionOptions_h__ */
