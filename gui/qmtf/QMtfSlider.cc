/*
 * QMtfSlider.cc
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#include "QMtfSlider.h"
#include <core/debug.h>

static constexpr int CTRL_HEIGHT = 16;
static constexpr int CTRL_WIDTH  = 16;



QMtfSlider::QMtfSlider(QWidget * parent)
  : Base(parent)
{
  setMinimumHeight(CTRL_HEIGHT);
  setMaximumHeight(CTRL_HEIGHT);

  sliders[SLIDER_SHADOWS].value = 0;
  sliders[SLIDER_HIGHLIGHTS].value = 1;
  sliders[SLIDER_MIDTONES].value = 0.5;

  resize(sizeHint());
}


QSize QMtfSlider::sizeHint() const
{
  return QSize(CTRL_HEIGHT, 256);
}

void QMtfSlider::setup(double shadows, double highlights, double midtones)
{
  sliders[SLIDER_SHADOWS].value = shadows;
  sliders[SLIDER_HIGHLIGHTS].value = highlights;

  sliders[SLIDER_MIDTONES].value =
      std::max(sliders[SLIDER_SHADOWS].value,
      std::min(midtones, sliders[SLIDER_HIGHLIGHTS].value));

  updateSliderRects();
  update();
}


void QMtfSlider::setShadows(double v)
{
  sliders[SLIDER_SHADOWS].value = v;
  updateSliderRects();
  update();
}

double QMtfSlider::shadows() const
{
  return sliders[SLIDER_SHADOWS].value;
}

void QMtfSlider::setHighlights(double v)
{
  sliders[SLIDER_HIGHLIGHTS].value = v;
  updateSliderRects();
  update();
}

double QMtfSlider::highlights() const
{
  return sliders[SLIDER_HIGHLIGHTS].value;
}

void QMtfSlider::setMidtones(double v)
{
  sliders[SLIDER_MIDTONES].value =
      std::max(sliders[SLIDER_SHADOWS].value,
      std::min(v, sliders[SLIDER_HIGHLIGHTS].value));

  updateSliderRects();
  update();
}

double QMtfSlider::midtones() const
{
  return sliders[SLIDER_MIDTONES].value;
}

void QMtfSlider::updateSliderRects()
{
  Slider & shadows = sliders[SLIDER_SHADOWS];
  Slider & highlights = sliders[SLIDER_HIGHLIGHTS];
  Slider & midtones = sliders[SLIDER_MIDTONES];

  shadows.rc.setRect(shadows.value * this->width() - CTRL_WIDTH / 2 - 1, 1,
      CTRL_WIDTH - 2, CTRL_HEIGHT - 2);

  highlights.rc.setRect(highlights.value * this->width() - CTRL_WIDTH / 2 - 1, 1,
      CTRL_WIDTH - 2, CTRL_HEIGHT - 2);

  midtones.rc.setRect(shadows.rc.x() + midtones.value * (highlights.rc.x() - shadows.rc.x()), 1,
      CTRL_WIDTH - 2, CTRL_HEIGHT - 2);
}

double QMtfSlider::computeSliderValue(int sliding_pos) const
{
  return (double)sliding_pos/ this->width();
}


void QMtfSlider::drawSlider(QPainter & p, int slider_side)
{
  Slider & s = sliders[slider_side];

  const QPoint points[] = {
      QPoint(s.rc.left(), s.rc.bottom()),
      QPoint(s.rc.right(), s.rc.bottom()),
      QPoint((s.rc.left() + s.rc.right()) / 2, s.rc.top()),
  };

  switch ( slider_side ) {
  case SLIDER_SHADOWS :
    p.setBrush(Qt::darkGray);
    break;
  case SLIDER_HIGHLIGHTS :
    p.setBrush(Qt::white);
    break;
  case SLIDER_MIDTONES :
    p.setBrush(Qt::lightGray);
    break;
  }

  p.setPen(Qt::black);
  p.drawConvexPolygon(points, 3);
}

void QMtfSlider::resizeEvent(QResizeEvent *e)
{
  Base::resizeEvent(e);
  updateSliderRects();
  update();
}

void QMtfSlider::paintEvent(QPaintEvent *e)
{
  QPainter p(this);

  const QSize size = this->size();

  p.fillRect(0, 0, size.width() - 1, size.height() - 1, QBrush(Qt::white));

  for ( uint i = 0; i < sizeof(sliders) / sizeof(sliders[0]); ++i ) {
    drawSlider(p, i);
  }
}

void QMtfSlider::mousePressEvent(QMouseEvent *e)
{
  if ( sliding_side < 0 ) {

    if ( e->buttons() == Qt::LeftButton  || e->buttons() == Qt::RightButton ) {

      const QPointF posf = e->localPos();
      const QPoint pos(posf.x(), posf.y());

      int search_order[3];

      if ( pos.x() < this->width() / 2  ) {
        search_order[0] = SLIDER_HIGHLIGHTS;
        search_order[1] = SLIDER_SHADOWS;
        search_order[2] = SLIDER_MIDTONES;
      }
      else {
        search_order[0] = SLIDER_SHADOWS;
        search_order[1] = SLIDER_HIGHLIGHTS;
        search_order[2] = SLIDER_MIDTONES;
      }


      int found_slider = -1;
      for ( uint j = 0; j < 3; ++j ) {

        const int i = search_order[j];

        if ( sliders[i].rc.contains(pos) ) {
          found_slider = i;
          break;
        }
      }

      if ( found_slider >= 0 ) {

        if ( e->buttons() == Qt::LeftButton  ) {
          sliding_side = found_slider;
          sliding_pos = pos.x();
          sliding_hit_offset = sliding_pos - sliders[found_slider].rc.x();
          return;
        }

        if ( e->buttons() == Qt::RightButton ) {

          switch ( found_slider ) {
          case SLIDER_SHADOWS :
            sliders[SLIDER_SHADOWS].value = 0.0;
            break;
          case SLIDER_HIGHLIGHTS :
            sliders[SLIDER_HIGHLIGHTS].value = 1.0;
            break;
          case SLIDER_MIDTONES :
            sliders[SLIDER_MIDTONES].value = 0.5;
            break;
          }
          updateSliderRects();
          update();
          emit mtfChanged();
          return;
        }
      }
    }
  }


  Base::mousePressEvent(e);
}

void QMtfSlider::mouseReleaseEvent(QMouseEvent *e)
{
  sliding_side = -1;
  Base::mouseReleaseEvent(e);
}

void QMtfSlider::mouseMoveEvent(QMouseEvent *e)
{
  int new_sliding_pos;
  double new_value;

  if ( sliding_side < 0 ) {
    Base::mouseMoveEvent(e);
    return;
  }

  if ( (new_sliding_pos = e->localPos().x()) == sliding_pos ) {
    return;
  }

  Slider & shadows = sliders[SLIDER_SHADOWS];
  Slider & highlights = sliders[SLIDER_HIGHLIGHTS];
  Slider & midtones = sliders[SLIDER_MIDTONES];

  if ( sliding_side != SLIDER_MIDTONES ) {
    if ( new_sliding_pos <= 0 || new_sliding_pos >= this->width() ) {
      return;
    }

    new_value = (double) (new_sliding_pos - sliding_hit_offset + CTRL_WIDTH / 2) / this->width();
  }
  else {

    if ( new_sliding_pos <= sliders[SLIDER_SHADOWS].rc.x() + CTRL_WIDTH / 2 ) {
      return;
    }
    if ( new_sliding_pos >= sliders[SLIDER_HIGHLIGHTS].rc.x() + CTRL_WIDTH / 2 ) {
      return;
    }

    const double relpos = new_sliding_pos - sliding_hit_offset - shadows.rc.x();
    new_value = std::max(0., std::min(1., relpos / (highlights.rc.x() - shadows.rc.x())));
  }


  const double old_shadows_value =
      sliders[SLIDER_SHADOWS].value;

  const double old_higlihts_value =
      sliders[SLIDER_HIGHLIGHTS].value;

  const double old_midtones_value =
      sliders[SLIDER_MIDTONES].value;

  if ( sliding_side == SLIDER_SHADOWS && new_value >= old_higlihts_value ) {
    return;
  }
  if ( sliding_side == SLIDER_HIGHLIGHTS && new_value <= old_shadows_value ) {
    return;
  }
  if ( sliding_side == SLIDER_MIDTONES && (new_value < 0 || new_value > 1 )) {
    return;
  }

  sliding_pos = new_sliding_pos;
  sliders[sliding_side].value = new_value;
  updateSliderRects();
  update();

  emit mtfChanged();
}
