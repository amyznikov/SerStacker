/*
 * QGraphicsRectShapeSettings.h
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsRectShapeSettings_h__
#define __QGraphicsRectShapeSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include "QGraphicsRectShape.h"

class QGraphicsRectShapeSettings :
    public QSettingsWidget
{
public:
  typedef QGraphicsRectShapeSettings ThisClass;
  typedef QSettingsWidget Base;

  QGraphicsRectShapeSettings(QWidget * parent = nullptr);
  QGraphicsRectShapeSettings(const QString &prefix, QWidget * parent = nullptr);

  void setShape(QGraphicsRectShape * shape);
  QGraphicsRectShape * shape() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  QGraphicsRectShape * shape_ = nullptr;
  QCheckBox * fixOnSceneCenter_ctl = nullptr;
  QColorPickerButton * penColor_ctl = nullptr;
  QSpinBox * penWidth_ctl = nullptr;
};



class QGraphicsRectShapeSettingsDialogBox:
    public QDialog
{
Q_OBJECT;
public:
  typedef QGraphicsRectShapeSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGraphicsRectShapeSettingsDialogBox(QWidget * parent = nullptr);

  QGraphicsRectShapeSettingsDialogBox(const QString & title,
      QWidget * parent = nullptr);

  QGraphicsRectShapeSettingsDialogBox(const QString & title, QGraphicsRectShape * shape,
      QWidget * parent = nullptr);

  void setShape(QGraphicsRectShape * shape);
  QGraphicsRectShape* shape() const;

  void loadParameters();

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QGraphicsRectShapeSettings *settigs_ctl = nullptr;
};

#endif /* __QGraphicsRectShapeSettings_h__ */
