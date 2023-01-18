/*
 * QColorPickerButton.h
 *
 *  Created on: Dec 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QColorPickerButton_h__
#define __QColorPickerButton_h__

#include <QtWidgets/QtWidgets>

class QColorPickerButton:
    public QPushButton
{
  Q_OBJECT;
public:
  typedef QColorPickerButton ThisClass;
  typedef QPushButton Base;

  QColorPickerButton(QWidget * parent = nullptr);
  QColorPickerButton(const QColor & color, QWidget * parent = nullptr);

  void setColor(const QColor & color);
  const QColor & color() const;

Q_SIGNALS:
  void colorSelected();

protected Q_SLOTS:
  void onClicked();

protected:
  void resizeEvent(QResizeEvent *event) override;
  void updateIcon();

protected:
  QColor color_ = Qt::white;
};

#endif /* __QColorPickerButton_h__ */
