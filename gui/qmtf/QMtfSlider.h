/*
 * QMtfSlider.h
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfSlider_h__
#define __QMtfSlider_h__

#include <QtWidgets/QtWidgets>

class QMtfSlider
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfSlider ThisClass;
  typedef QWidget Base;

  QMtfSlider(QWidget * parent = Q_NULLPTR);

  void setup(double shadows, double highlights, double midtones);

  void setShadows(double v);
  double shadows() const;

  void setMidtones(double v);
  double midtones() const;

  void setHighlights(double v);
  double highlights() const;


signals:
  void mtfChanged();

protected:
  QSize sizeHint() const override;
  void paintEvent(QPaintEvent *e) override;
  void resizeEvent(QResizeEvent *e) override;
  void mousePressEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void drawSlider(QPainter & p, int slider_side);
  void updateSliderRects();
  double computeSliderValue(int sliding_pos) const;

protected:

  enum {
    SLIDER_SHADOWS = 0,
    SLIDER_MIDTONES = 1,
    SLIDER_HIGHLIGHTS = 2,
  };

  struct Slider {
    double value = 0;
    QRect rc;
  };

  Slider sliders[3];
  int sliding_side = -1;
  int sliding_pos = 0;
  int sliding_hit_offset = 0;
};

#endif /* __QMtfSlider_h__ */
