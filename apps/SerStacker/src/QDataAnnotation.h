/*
 * QDataAnnotation.h
 *
 *  Created on: Oct 23, 2024
 *      Author: gandriim
 */

#ifndef __QDataAnnotation_h__
#define __QDataAnnotation_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QSettingsWidgetTemplate.h>
#include <core/data_annotation/c_data_annotation_labels.h>

namespace serstacker {

class QDataAnnotationSettingsWidget :
    public QSettingsWidgetTemplate<c_data_annotation_labels>
{
public:
  typedef QDataAnnotationSettingsWidget ThisClass;
  typedef QSettingsWidgetTemplate<c_data_annotation_labels> Base;

  QDataAnnotationSettingsWidget(QWidget * parent = nullptr);

  void setOpts(OptsType * opts) final;

protected:
  void populateColormaps();
  void onCurrentColormapMapChanged(int cursel);
  void onCurrentLabelSelectionChanged(int cursel);

protected:
  QComboBox * colorMapSelection_ctl = nullptr;
  QComboBox * labelSelection_ctl = nullptr;
  QSpinBox * labelValue_ctl  = nullptr;
  QColorPickerButton * labelColor_ctl = nullptr;
  QDoubleSpinBox * labelBlendAlpha_ctl = nullptr;

};

class QDataAnnotationsSettingsDialogBox :
    public QSettingsDialogBoxTemplate<QDataAnnotationSettingsWidget>
{
public:
  typedef QDataAnnotationsSettingsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QDataAnnotationSettingsWidget> Base;

  QDataAnnotationsSettingsDialogBox(QWidget * parent = nullptr) :
    Base("Data Annotation Options", parent)
  {
  }
};



} /* namespace serstacker */

#endif /* __QDataAnnotation_h__ */
