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
    SLIDER_SHADOWS = 3,
    SLIDER_HIGHLIGHTS = 4,
  };

  QMtfSliderBand(QWidget * parent = nullptr);

  void setOpts(double lclip, double hclip, double midtones, double shadows, double highlights);

  void setlclipRange(double minv, double maxv, double v);
  void setlclip(double v);
  double lclip() const;

  void sethclipRange(double minv, double maxv, double v);
  void sethclip(double v);
  double hclip() const;

  void setMidtonesRange(double minv, double maxv, double v);
  void setMidtones(double v);
  double midtones() const;

  void setShadowsRange(double minv, double maxv, double v);
  void setShadows(double v);
  double shadows() const;

  void setHighlightsRange(double minv, double maxv, double v);
  void setHighlights(double v);
  double highlights() const;

Q_SIGNALS:
  void positonChanged(int slider, double value);

protected:

  struct Slider {
    QRect rc;
    QRect brc;
    double v = 0;
    double minv = 0;
    double maxv = 1;
  };

  QSize sizeHint() const final;
  void paintEvent(QPaintEvent *e) final;
  void resizeEvent(QResizeEvent *e) final;
  void mousePressEvent(QMouseEvent *e) final;
  void mouseReleaseEvent(QMouseEvent *e) final;
  void mouseMoveEvent(QMouseEvent *e) final;
  void updateSliderRects();
  double computeSliderValue(int sliding_pos) const;
  int sliderIndex(const Slider * s) const;

protected:
  Slider _lclip;
  Slider _hclip;
  Slider _midtones;
  Slider _shadows;
  Slider _highlights;

  Slider * _currentSlider = nullptr;

  int sliding_mouse_pos = 0;
  int sliding_hit_offset = 0;
};

#endif /* __QMtfSlider_h__ */
