/*
 * QGraphicsTargetShapeSettings.h
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsTargetShapeSettings_h__
#define __QGraphicsTargetShapeSettings_h__

#include "QGraphicsTargetShape.h"
#include "QGraphicsShapeSettings.h"
#include <gui/widgets/QColorPickerButton.h>

class QGraphicsTargetShapeSettings:
    public QGraphicsShapeSettings<QGraphicsTargetShape>
{
public:
  typedef QGraphicsTargetShapeSettings ThisClass;
  typedef QGraphicsShapeSettings<QGraphicsTargetShape> Base;

  QGraphicsTargetShapeSettings(QWidget * parent = nullptr);

protected:
  void onload(const QSettings & settings, const QString & prefix = "") override;
  void onsave(QSettings & settings, const QString & prefix = "") override;

protected:
  QCheckBox * lockPosition_ctl = nullptr;
  QCheckBox * fixOnSceneCenter_ctl = nullptr;
  QSpinBox * numRings_ctl = nullptr;
  QIntegerSliderSpinBox * baseRadius_ctl = nullptr;
  QCheckBox * showDiagonalRays_ctl = nullptr;
  QColorPickerButton * penColor_ctl = nullptr;
  QSpinBox * penWidth_ctl = nullptr;
};

class QGraphicsTargetShapeSettingsDialogBox:
    public QGraphicsShapeSettingsDialogBox<QGraphicsTargetShapeSettings>
{
Q_OBJECT;
public:
  typedef QGraphicsTargetShapeSettingsDialogBox ThisClass;
  typedef QGraphicsShapeSettingsDialogBox<QGraphicsTargetShapeSettings> Base;

  QGraphicsTargetShapeSettingsDialogBox(QWidget * parent = nullptr) :
    ThisClass("Shape Options", parent)
  {
  }

  QGraphicsTargetShapeSettingsDialogBox(const QString & title, QWidget * parent = nullptr) :
    ThisClass("Shape Options", nullptr, parent)
  {
  }

  QGraphicsTargetShapeSettingsDialogBox(const QString & title, QGraphicsTargetShape * shape, QWidget * parent = nullptr) :
    Base(title, parent)
  {
    setShape(shape);
  }
};

#endif /* __QGraphicsTargetShapeSettings_h__ */
