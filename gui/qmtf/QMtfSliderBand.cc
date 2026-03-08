/*
 * QMtfSliderStrip.cc
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#include "QMtfSliderBand.h"

#include <core/debug.h>

static constexpr int CTRL_HEIGHT = 16;
static constexpr int CTRL_WIDTH  = 16;



QMtfSliderBand::QMtfSliderBand(QWidget * parent) :
    Base(parent)
{
  setMinimumHeight(CTRL_HEIGHT);
  setMaximumHeight(CTRL_HEIGHT);

  _lclip.value = 0;
  _hclip.value = 1;
  _midtones.value = 0.5;

  resize(sizeHint());
}

QSize QMtfSliderBand::sizeHint() const
{
  return QSize(CTRL_HEIGHT, 256);
}

void QMtfSliderBand::setOpts(double lclip, double hclip, double midtones)
{
  _lclip.value = lclip;
  _hclip.value = hclip;
  _midtones.value = midtones;

  updateSliderRects();
  update();
}

int QMtfSliderBand::sliderIndex(const Slider * s) const
{
  if ( s == &_lclip ) {
    return SLIDER_LCLIP;
  }
  if ( s == &_hclip ) {
    return SLIDER_HCLIP;
  }
  if ( s == &_midtones ) {
    return SLIDER_MIDTONES;
  }
  return -1;
}

void QMtfSliderBand::setlclip(double v)
{
  _lclip.value = v;
  updateSliderRects();
  update();
}

double QMtfSliderBand::lclip() const
{
  return _lclip.value;
}

void QMtfSliderBand::sethclip(double v)
{
  _hclip.value = v;
  updateSliderRects();
  update();
}

double QMtfSliderBand::hclip() const
{
  return _hclip.value;
}

void QMtfSliderBand::setMidtones(double v)
{
  _midtones.value = v;
  updateSliderRects();
  update();
}

double QMtfSliderBand::midtones() const
{
  return _midtones.value;
}

void QMtfSliderBand::updateSliderRects()
{
  _lclip.rc.setRect(_lclip.value * this->width() - CTRL_WIDTH / 2 - 1, 1, CTRL_WIDTH - 2, CTRL_HEIGHT - 2);
  _hclip.rc.setRect(_hclip.value * this->width() - CTRL_WIDTH / 2 - 1, 1, CTRL_WIDTH - 2, CTRL_HEIGHT - 2);
  _midtones.rc.setRect(_midtones.value * this->width() - CTRL_WIDTH / 2 - 1, 1, CTRL_WIDTH - 2, CTRL_HEIGHT - 2);
}

double QMtfSliderBand::computeSliderValue(int sliding_pos) const
{
  return (double)sliding_pos/ this->width();
}


void QMtfSliderBand::drawSlider(QPainter & p, const Slider & s, const QColor & color)
{
  const QPoint points[] = {
      QPoint(s.rc.left(), s.rc.bottom()),
      QPoint(s.rc.right(), s.rc.bottom()),
      QPoint((s.rc.left() + s.rc.right()) / 2, s.rc.top()),
  };


//  switch ( slider_side ) {
//  case SLIDER_SHADOWS :
//    p.setBrush(Qt::darkGray);
//    break;
//  case SLIDER_HIGHLIGHTS :
//    p.setBrush(Qt::white);
//    break;
//  case SLIDER_MIDTONES :
//    p.setBrush(Qt::lightGray);
//    break;
//  }

  p.setBrush(color);
  p.setPen(Qt::black);
  p.drawConvexPolygon(points, 3);
}

void QMtfSliderBand::resizeEvent(QResizeEvent *e)
{
  Base::resizeEvent(e);
  updateSliderRects();
  update();
}

void QMtfSliderBand::paintEvent(QPaintEvent *e)
{
  QPainter p(this);

  const QSize size = this->size();

  p.fillRect(0, 0, size.width() - 1, size.height() - 1, QBrush(Qt::white));
  drawSlider(p, _lclip, Qt::black);
  drawSlider(p, _hclip, Qt::white);
  drawSlider(p, _midtones, Qt::lightGray);
}

void QMtfSliderBand::mousePressEvent(QMouseEvent *e)
{
  if ( !_currentSlider ) {

    if ( e->buttons() == Qt::LeftButton  || e->buttons() == Qt::RightButton ) {

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
      const QPointF posf = e->position();
#else
      const QPointF posf = e->localPos();
#endif
      const QPoint pos(posf.x(), posf.y());

      Slider * search_order[3] = {nullptr};

      if ( pos.x() < this->width() / 2  ) {
        search_order[0] = &_hclip;
        search_order[1] = &_lclip;
        search_order[2] = &_midtones;
      }
      else {
        search_order[0] = &_lclip;
        search_order[1] = &_hclip;
        search_order[2] = &_midtones;
      }

      Slider * found_slider = nullptr;
      for ( uint j = 0; j < 3; ++j ) {
        if ( search_order[j]->rc.contains(pos) ) {
          found_slider = search_order[j];
          break;
        }
      }

      if ( found_slider ) {

        if ( e->buttons() == Qt::LeftButton  ) {
          _currentSlider = found_slider;
          sliding_pos = pos.x();
          sliding_hit_offset = sliding_pos - found_slider->rc.x();
          return;
        }

        if ( e->buttons() == Qt::RightButton ) {

          int slider_index = -1;

          if ( found_slider == &_lclip ) {
            found_slider->value = 0;
            slider_index = SLIDER_LCLIP;
          }
          else if ( found_slider == &_hclip ) {
            found_slider->value = 1;
            slider_index = SLIDER_HCLIP;
          }
          else if ( found_slider == &_midtones ) {
            found_slider->value = 0.5;
            slider_index = SLIDER_MIDTONES;
          }

          if ( slider_index >= 0 ) {
            updateSliderRects();
            update();
            Q_EMIT positonChanged(slider_index, found_slider->value);
          }
          return;
        }
      }
    }
  }

  Base::mousePressEvent(e);
}

void QMtfSliderBand::mouseReleaseEvent(QMouseEvent *e)
{
  _currentSlider = nullptr;
  Base::mouseReleaseEvent(e);
}

void QMtfSliderBand::mouseMoveEvent(QMouseEvent *e)
{
  if ( !_currentSlider ) {
    Base::mouseMoveEvent(e);
    return;
  }

  int new_sliding_pos;
  double new_value;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  if ( (new_sliding_pos = e->position().x()) == sliding_pos ) {
    return;
  }
#else
  if ( (new_sliding_pos = e->localPos().x()) == sliding_pos ) {
    return;
  }
#endif

  const int w  = this->width();

  if ( new_sliding_pos < 0 || new_sliding_pos >= w ) {
    return;
  }

  new_value = (double) (new_sliding_pos - sliding_hit_offset + CTRL_WIDTH / 2) / w;
  //  const double relpos = new_sliding_pos - sliding_hit_offset - shadows.rc.x();
  //  new_value = std::max(0., std::min(1., relpos / (highlights.rc.x() - shadows.rc.x())));

  sliding_pos = new_sliding_pos;
  _currentSlider->value = new_value;
  updateSliderRects();
  update();

  Q_EMIT positonChanged(sliderIndex(_currentSlider),  _currentSlider->value);
}
