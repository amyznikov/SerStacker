/*
 * QGraphicsTargetShapeSettings.h
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsTargetShapeSettings_h__
#define __QGraphicsTargetShapeSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include "QGraphicsTargetShape.h"

class QGraphicsTargetShapeSettings:
    public QSettingsWidget
{
public:
  typedef QGraphicsTargetShapeSettings ThisClass;
  typedef QSettingsWidget Base;

  QGraphicsTargetShapeSettings(QWidget * parent = nullptr);
  QGraphicsTargetShapeSettings(const QString &prefix, QWidget * parent = nullptr);

  void setShape(QGraphicsTargetShape * shape);
  QGraphicsTargetShape * shape() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  QGraphicsTargetShape * shape_ = nullptr;
  QCheckBox * fixOnSceneCenter_ctl = nullptr;
  QNumberEditBox * numRings_ctl = nullptr;
  QNumberEditBox * baseRadius_ctl = nullptr;
  QCheckBox * showDiagonalRays_ctl = nullptr;
  QColorPickerButton * penColor_ctl = nullptr;
  QSpinBox * penWidth_ctl = nullptr;
};

class QGraphicsTargetShapeSettingsDialogBox:
    public QDialog
{
Q_OBJECT;
public:
  typedef QGraphicsTargetShapeSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGraphicsTargetShapeSettingsDialogBox(QWidget * parent = nullptr);

  QGraphicsTargetShapeSettingsDialogBox(const QString & title,
      QWidget * parent = nullptr);

  QGraphicsTargetShapeSettingsDialogBox(const QString & title, QGraphicsTargetShape * shape,
      QWidget * parent = nullptr);

  void setShape(QGraphicsTargetShape * shape);
  QGraphicsTargetShape* shape() const;

  void loadParameters();

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QGraphicsTargetShapeSettings *settigs_ctl = nullptr;
};

#endif /* __QGraphicsTargetShapeSettings_h__ */
