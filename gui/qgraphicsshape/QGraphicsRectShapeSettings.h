/*
 * QGraphicsRectShapeSettings.h
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsRectShapeSettings_h__
#define __QGraphicsRectShapeSettings_h__

#include "QGraphicsRectShape.h"
#include "QGraphicsShapeSettings.h"
#include <gui/widgets/QColorPickerButton.h>

class QGraphicsRectShapeSettings :
    public QGraphicsShapeSettings<QGraphicsRectShape>
{
public:
  typedef QGraphicsRectShapeSettings ThisClass;
  typedef QGraphicsShapeSettings<QGraphicsRectShape> Base;

  QGraphicsRectShapeSettings(QWidget * parent = nullptr);

protected:
  void onload(const QSettings & settings, const QString & prefix = "") override;
  void onsave(QSettings & settings, const QString & prefix = "") override;

protected:
  QCheckBox * snapToPixelGrid_ctl = nullptr;
  QCheckBox * fixOnSceneCenter_ctl = nullptr;
  QColorPickerButton * penColor_ctl = nullptr;
  QSpinBox * penWidth_ctl = nullptr;
  QNumericBox * rect_ctl = nullptr;
};



class QGraphicsRectShapeSettingsDialogBox:
    public QGraphicsShapeSettingsDialogBox<QGraphicsRectShapeSettings>
{
  Q_OBJECT;
public:
  typedef QGraphicsRectShapeSettingsDialogBox ThisClass;
  typedef QGraphicsShapeSettingsDialogBox<QGraphicsRectShapeSettings> Base;

  QGraphicsRectShapeSettingsDialogBox(QWidget * parent = nullptr) :
      ThisClass("Shape Options", parent)
  {
  }

  QGraphicsRectShapeSettingsDialogBox(const QString & title, QWidget * parent = nullptr) :
      ThisClass("Shape Options", nullptr, parent)
  {
  }

  QGraphicsRectShapeSettingsDialogBox(const QString & title, QGraphicsRectShape * shape, QWidget * parent = nullptr) :
      Base(title, parent)
  {
    layout()->setSizeConstraint(QLayout::SetFixedSize);
    _settings->setShape(shape);
  }
};

#endif /* __QGraphicsRectShapeSettings_h__ */
