/*
 * QMtfSliderStrip.cc
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#include "QMtfSliderBand.h"

#include <core/debug.h>

static constexpr int CTRL_HEIGHT1 = 14;
static constexpr int CTRL_HEIGHT2 = 10;
static constexpr int CTRL_WIDTH1  = 14;
static constexpr int CTRL_WIDTH2  = 12;



QMtfSliderBand::QMtfSliderBand(QWidget * parent) :
    Base(parent)
{
  setMinimumHeight(CTRL_HEIGHT1 + CTRL_HEIGHT2 + 2);
  setMaximumHeight(CTRL_HEIGHT1 + CTRL_HEIGHT2 + 2);

  _lclip.v = 0;
  _hclip.v = 1;
  _midtones.v = 0.5;
  _shadows.v = 0.0;
  _highlights.v = 0.0;

  resize(sizeHint());
}

QSize QMtfSliderBand::sizeHint() const
{
  return QSize(256, minimumHeight());
}

void QMtfSliderBand::resizeEvent(QResizeEvent *e)
{
  Base::resizeEvent(e);
  updateSliderRects();
  update();
}

void QMtfSliderBand::setOpts(double lclip, double hclip, double midtones, double shadows, double highlights)
{
  _lclip.v = std::clamp(lclip, _lclip.minv, _lclip.maxv);
  _hclip.v = std::clamp(hclip, _hclip.minv, _hclip.maxv);
  _midtones.v = std::clamp(midtones, _midtones.minv, _midtones.maxv);
  _shadows.v = std::clamp(shadows, _shadows.minv, _shadows.maxv);
  _highlights.v = std::clamp(highlights, _highlights.minv, _highlights.maxv);
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
  if ( s == &_shadows ) {
    return SLIDER_SHADOWS;
  }
  if ( s == &_highlights ) {
    return SLIDER_HIGHLIGHTS;
  }
  return -1;
}

void QMtfSliderBand::setlclipRange(double minv, double maxv, double v)
{
  _lclip.minv = minv;
  _lclip.maxv = maxv;
  _lclip.v = std::clamp(v, minv, maxv);
  updateSliderRects();
  update();
}

void QMtfSliderBand::setlclip(double v)
{
  _lclip.v = std::clamp(v, _lclip.minv, _lclip.maxv);
  updateSliderRects();
  update();
}

double QMtfSliderBand::lclip() const
{
  return _lclip.v;
}

void QMtfSliderBand::sethclipRange(double minv, double maxv, double v)
{
  _hclip.minv = minv;
  _hclip.maxv = maxv;
  _hclip.v = std::clamp(v, minv, maxv);
  updateSliderRects();
  update();
}

void QMtfSliderBand::sethclip(double v)
{
  _hclip.v = std::clamp(v, _hclip.minv, _hclip.maxv);
  updateSliderRects();
  update();
}

double QMtfSliderBand::hclip() const
{
  return _hclip.v;
}

void QMtfSliderBand::setMidtonesRange(double minv, double maxv, double v)
{
  _midtones.minv = minv;
  _midtones.maxv = maxv;
  _midtones.v = std::clamp(v, minv, maxv);
  updateSliderRects();
  update();
}

void QMtfSliderBand::setMidtones(double v)
{
  _midtones.v = std::clamp(v, _midtones.minv, _midtones.maxv);
  updateSliderRects();
  update();
}

double QMtfSliderBand::midtones() const
{
  return _midtones.v;
}

void QMtfSliderBand::setShadowsRange(double minv, double maxv, double v)
{
  _shadows.minv = minv;
  _shadows.maxv = maxv;
  _shadows.v = std::clamp(v, minv, maxv);
  updateSliderRects();
  update();
}

void QMtfSliderBand::setShadows(double v)
{
  _shadows.v = std::clamp(v, _shadows.minv, _shadows.maxv);
  updateSliderRects();
  update();

}

double QMtfSliderBand::shadows() const
{
  return _shadows.v;
}

void QMtfSliderBand::setHighlightsRange(double minv, double maxv, double v)
{
  _highlights.minv = minv;
  _highlights.maxv = maxv;
  _highlights.v = std::clamp(v, minv, maxv);
  updateSliderRects();
  update();
}

void QMtfSliderBand::setHighlights(double v)
{
  _highlights.v = std::clamp(v, _highlights.minv, _highlights.maxv);
  updateSliderRects();
  update();
}

double QMtfSliderBand::highlights() const
{
  return _highlights.v;
}

void QMtfSliderBand::updateSliderRects()
{
  const int w = width();
  const int h = height();
  const int w2 = w / 2;
  const int h1 = CTRL_HEIGHT1;
  const int h2 = CTRL_HEIGHT2;

  _lclip.brc.setRect(0, 0, w, h1);
  _hclip.brc.setRect(0, 0, w, h1);
  _midtones.brc.setRect(0,0, w, h1);
  _shadows.brc.setRect(0, h - h2, w2, h2);
  _highlights.brc.setRect(w2, h - h2, w2, h2);

  static const auto updateRC =
      [](Slider & s, int cw, int ch) {
        const int w = s.brc.width();
        s.rc.setRect(s.brc.x() + (s.v - s.minv) * w / (s.maxv - s.minv) - cw / 2, s.brc.bottom() - ch, cw, ch);
      };

  updateRC(_lclip, CTRL_WIDTH1, CTRL_HEIGHT1);
  updateRC(_midtones, CTRL_WIDTH1, CTRL_HEIGHT1);
  updateRC(_hclip, CTRL_WIDTH1, CTRL_HEIGHT1);
  updateRC(_shadows, CTRL_WIDTH2, CTRL_HEIGHT2);
  updateRC(_highlights, CTRL_WIDTH2, CTRL_HEIGHT2);
}

void QMtfSliderBand::paintEvent(QPaintEvent * e)
{
  QPainter p(this);

  static const auto drawSlider =
      [](QPainter & p, const Slider & s, const QColor & color) {
        const QPoint points[] = {
          QPoint(s.rc.left(), s.rc.bottom()),
          QPoint(s.rc.right(), s.rc.bottom()),
          QPoint((s.rc.left() + s.rc.right()) / 2, s.rc.top()),
        };

        p.setBrush(color);
        p.setPen(Qt::black);
        p.drawConvexPolygon(points, 3);
      };

  const QSize size = this->size();
  p.fillRect(0, 0, size.width() - 1, size.height() - 1, QBrush(Qt::white));
  p.setPen(Qt::darkGray);
  p.drawLine(size.width() / 2, size.height(), size.width() / 2, size.height() - CTRL_HEIGHT2 - 1);
  p.drawLine(0, size.height() - CTRL_HEIGHT2 - 2, size.width(), size.height() - CTRL_HEIGHT2 - 2);
  //p.drawLine(0, size.height()-1, size.width(), size.height()-1);

  drawSlider(p, _lclip, Qt::black);
  drawSlider(p, _hclip, Qt::white);
  drawSlider(p, _midtones, Qt::lightGray);
  drawSlider(p, _shadows, Qt::darkGray);
  drawSlider(p, _highlights, Qt::lightGray);
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

      Slider * search_order[5] = {nullptr};

      const int w  = this->width();

      if ( pos.x() < w / 2  ) {
        search_order[0] = &_hclip;
        search_order[1] = &_lclip;
        search_order[2] = &_midtones;
        search_order[3] = &_shadows;
        search_order[4] = &_highlights;
      }
      else {
        search_order[0] = &_lclip;
        search_order[1] = &_hclip;
        search_order[2] = &_midtones;
        search_order[3] = &_highlights;
        search_order[4] = &_shadows;
      }

      Slider * found_slider = nullptr;
      for ( uint j = 0; j < 5; ++j ) {
        if ( search_order[j]->rc.contains(pos) ) {
          found_slider = search_order[j];
          break;
        }
      }

      if ( found_slider ) {

        if ( e->buttons() == Qt::LeftButton  ) {
          _currentSlider = found_slider;
          sliding_mouse_pos = pos.x();
          sliding_hit_offset = sliding_mouse_pos - found_slider->rc.x();
          return;
        }

        if ( e->buttons() == Qt::RightButton ) {

          int slider_index = -1;

          if ( found_slider == &_lclip ) {
            found_slider->v = 0;
            slider_index = SLIDER_LCLIP;
          }
          else if ( found_slider == &_hclip ) {
            found_slider->v = 1;
            slider_index = SLIDER_HCLIP;
          }
          else if ( found_slider == &_midtones ) {
            found_slider->v = 0.5;
            slider_index = SLIDER_MIDTONES;
          }
          else if ( found_slider == &_shadows ) {
            found_slider->v = 0;
            slider_index = SLIDER_SHADOWS;
          }
          else if ( found_slider == &_highlights ) {
            found_slider->v = 0;
            slider_index = SLIDER_HIGHLIGHTS;
          }

          if ( slider_index >= 0 ) {
            updateSliderRects();
            update();
            Q_EMIT positonChanged(slider_index, found_slider->v);
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

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  if ( (new_sliding_pos = e->position().x()) == sliding_mouse_pos ) {
    return;
  }
#else
  if ( (new_sliding_pos = e->localPos().x()) == sliding_mouse_pos ) {
    return;
  }
#endif

  if( new_sliding_pos < _currentSlider->brc.left() || new_sliding_pos >= _currentSlider->brc.right() ) {
    return;
  }

  sliding_mouse_pos = new_sliding_pos;

  const double relative_offset = (new_sliding_pos - _currentSlider->brc.left()) / (double) _currentSlider->brc.width();
  _currentSlider->v = std::clamp(_currentSlider->minv + relative_offset * (_currentSlider->maxv - _currentSlider->minv),
      _currentSlider->minv, _currentSlider->maxv);

  updateSliderRects();
  update();

  Q_EMIT positonChanged(sliderIndex(_currentSlider),  _currentSlider->v);
}
