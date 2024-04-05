/*
 * QScaleSelectionButton.h
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#ifndef __QScaleSelectionButton_h__
#define __QScaleSelectionButton_h__

#include <QtWidgets/QtWidgets>

class QScaleSelectionButton :
    public QToolButton
{
  Q_OBJECT;
public:
  typedef QScaleSelectionButton ThisClass;
  typedef QToolButton Base;

  QScaleSelectionButton(QWidget * parent = nullptr);

  int currentScale() const;

public Q_SLOTS:
  void setScaleRange(int min, int max);
  void setCurrentScale(int scale);

Q_SIGNALS:
  void scaleChanged(int currentScale);

protected:
  class QPopupSlider;
  QPopupSlider * popup_ = nullptr;
};

#endif /* __QScaleSelectionButton_h__ */
