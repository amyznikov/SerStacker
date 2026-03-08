/*
 * QMtfSliderStrip.h
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfSlider_h__
#define __QMtfSlider_h__

#include <QtWidgets/QtWidgets>

class QMtfSliderBand :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfSliderBand ThisClass;
  typedef QWidget Base;

  enum SLIDER_INDEX {
    SLIDER_LCLIP = 0,
    SLIDER_HCLIP = 1,
    SLIDER_MIDTONES = 2,
  };

  QMtfSliderBand(QWidget * parent = nullptr);

  void setOpts(double lclip, double hclip, double midtones);

  void setlclip(double v);
  double lclip() const;

  void sethclip(double v);
  double hclip() const;

  void setMidtones(double v);
  double midtones() const;

Q_SIGNALS:
  void positonChanged(int slider, double value);

protected:

  struct Slider {
    QRect rc;
    double value = 0;
  };

  QSize sizeHint() const final;
  void paintEvent(QPaintEvent *e) final;
  void resizeEvent(QResizeEvent *e) final;
  void mousePressEvent(QMouseEvent *e) final;
  void mouseReleaseEvent(QMouseEvent *e) final;
  void mouseMoveEvent(QMouseEvent *e) final;
  void drawSlider(QPainter & p, const Slider & slider, const QColor & color);
  void updateSliderRects();
  double computeSliderValue(int sliding_pos) const;
  int sliderIndex(const Slider * s) const;

protected:
  Slider _lclip;
  Slider _hclip;
  Slider _midtones;
  Slider * _currentSlider = nullptr;

  int sliding_pos = 0;
  int sliding_hit_offset = 0;
};

#endif /* __QMtfSlider_h__ */
