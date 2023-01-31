/*
 * QProgressStrip.h
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QProgressStrip_h_
#define __QProgressStrip_h_

#include <QtWidgets/QtWidgets>

class QProgressStrip:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QProgressStrip ThisClass;
  typedef QWidget Base;

  struct Strip {
    QBrush brush;
    double value = 0;
  };

  QProgressStrip(QWidget * parent = nullptr);

  void setMinValue(double v);
  double minValue() const;

  void setMaxValue(double v);
  double maxValue() const;

  void setRange(double minValue, double maxValue);
  void getRange(double *minValue, double *maxValue) const;

  void setNumStrips(int v);
  int numStrips() const;

  void setValue(int stripIndex, double v);
  double value(int stripIndex) const;

  void setBrush(int stripIndex, const QBrush & brush);
  const QBrush & brush(int stripIndex);

  void setBackgroundColor(const QBrush & brush);
  const QBrush & backgroundColor();

  void setTextColor(const QColor & color);
  const QColor & textColor();

  void setText(const QString & text);
  const QString & text() const;

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent *event) override;

protected:
  double minValue_ = 0;
  double maxValue_ = 100;
  std::vector<Strip> strips_;
  QBrush bgBrush_ = QBrush(Qt::white);
  QColor textColor_ = Qt::black;
  QString text_;
};

#endif /* __QProgressStrip_h_ */
