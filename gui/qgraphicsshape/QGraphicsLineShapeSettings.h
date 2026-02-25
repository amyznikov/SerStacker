/*
 * QGraphicsLineShapeSettings.h
 *
 *  Created on: Jan 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsLineShapeSettings_h__
#define __QGraphicsLineShapeSettings_h__

#include "QGraphicsLineShape.h"
#include "QGraphicsShapeSettings.h"
#include <gui/widgets/QColorPickerButton.h>

class QGraphicsLineShapeSettings:
    public QGraphicsShapeSettings<QGraphicsLineShape>
{
public:
  typedef QGraphicsLineShapeSettings ThisClass;
  typedef QGraphicsShapeSettings<QGraphicsLineShape> Base;

  QGraphicsLineShapeSettings(QWidget * parent = nullptr);

protected:
  void onload(const QSettings & settings, const QString & prefix = "") override;
  void onsave(QSettings & settings, const QString & prefix = "") override;

protected:
  QCheckBox * lockP1_ctl = nullptr;
  QCheckBox * lockP2_ctl = nullptr;
  QCheckBox * snapToPixelGrid_ctl = nullptr;
  QColorPickerButton *penColor_ctl = nullptr;
  QSpinBox *penWidth_ctl = nullptr;
  QSpinBox *arrowSize_ctl = nullptr;
};


class QGraphicsLineShapeSettingsDialogBox:
    public QGraphicsShapeSettingsDialogBox<QGraphicsLineShapeSettings>
{
Q_OBJECT;
public:
  typedef QGraphicsLineShapeSettingsDialogBox ThisClass;
  typedef QGraphicsShapeSettingsDialogBox<QGraphicsLineShapeSettings> Base;

  QGraphicsLineShapeSettingsDialogBox(QWidget * parent = nullptr) :
    ThisClass("Shape Options", parent)
  {
  }

  QGraphicsLineShapeSettingsDialogBox(const QString & title, QWidget * parent = nullptr) :
    ThisClass("Shape Options", nullptr, parent)
  {
  }

  QGraphicsLineShapeSettingsDialogBox(const QString & title, QGraphicsLineShape * shape, QWidget * parent = nullptr) :
    Base(title, parent)
  {
    setShape(shape);
  }
};

#endif /* __QGraphicsLineShapeSettings_h__ */
