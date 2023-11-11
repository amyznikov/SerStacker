/*
 * QGraphicsLineShapeSettings.h
 *
 *  Created on: Jan 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsLineShapeSettings_h__
#define __QGraphicsLineShapeSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include "QGraphicsLineShape.h"

class QGraphicsLineShapeSettings:
    public QSettingsWidget
{
public:
  typedef QGraphicsLineShapeSettings ThisClass;
  typedef QSettingsWidget Base;

  QGraphicsLineShapeSettings(QWidget * parent = nullptr);
  QGraphicsLineShapeSettings(const QString & prefix, QWidget * parent = nullptr);

  void setShape(QGraphicsLineShape * shape);
  QGraphicsLineShape* shape() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  QGraphicsLineShape *shape_ = nullptr;
  QCheckBox * lockP1_ctl = nullptr;
  QCheckBox * lockP2_ctl = nullptr;
  QCheckBox * snapToPixelGrid_ctl = nullptr;
  QColorPickerButton *penColor_ctl = nullptr;
  QSpinBox *penWidth_ctl = nullptr;
  QSpinBox *arrowSize_ctl = nullptr;
};


class QGraphicsLineShapeSettingsDialogBox:
    public QDialog
{
Q_OBJECT;
public:
  typedef QGraphicsLineShapeSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGraphicsLineShapeSettingsDialogBox(QWidget * parent = nullptr);

  QGraphicsLineShapeSettingsDialogBox(const QString & title,
      QWidget * parent = nullptr);

  QGraphicsLineShapeSettingsDialogBox(const QString & title, QGraphicsLineShape * shape,
      QWidget * parent = nullptr);

  void setShape(QGraphicsLineShape * shape);
  QGraphicsLineShape* shape() const;

  void loadParameters();

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QGraphicsLineShapeSettings *settigs_ctl = nullptr;
};

#endif /* __QGraphicsLineShapeSettings_h__ */
