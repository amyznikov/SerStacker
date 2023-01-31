/*
 * QProgressStrip.cc
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#include "QProgressStrip.h"
#include <core/debug.h>

QProgressStrip::QProgressStrip(QWidget * parent) :
  Base(parent)
{
}

void QProgressStrip::setMinValue(double v)
{
  minValue_ = v;
  update();
}

double QProgressStrip::minValue() const
{
  return minValue_;
}

void QProgressStrip::setMaxValue(double v)
{
  maxValue_ = v;
  update();
}

double QProgressStrip::maxValue() const
{
  return maxValue_;
}

void QProgressStrip::setRange(double minValue, double maxValue)
{
  minValue_ = minValue;
  maxValue_ = maxValue;
  update();
}


void QProgressStrip::getRange(double *minValue, double *maxValue) const
{
  *minValue = minValue_;
  *maxValue = maxValue_;
}

void QProgressStrip::setNumStrips(int v)
{
  strips_.resize(v);
}

int QProgressStrip::numStrips() const
{
  return strips_.size();
}

void QProgressStrip::setValue(int stripIndex, double v)
{
  if( stripIndex >= 0 && stripIndex < (int) strips_.size() ) {
    strips_[stripIndex].value = v;
    update();
  }
}

double QProgressStrip::value(int stripIndex) const
{
  return strips_[stripIndex].value;
}

void QProgressStrip::setBrush(int stripIndex, const QBrush & brush)
{
  if( stripIndex >= 0 && stripIndex < (int) strips_.size() ) {
    strips_[stripIndex].brush = brush;
    update();
  }
}

const QBrush & QProgressStrip::brush(int stripIndex)
{
  return strips_[stripIndex].brush;
}


void QProgressStrip::setBackgroundColor(const QBrush & brush)
{
  bgBrush_ = brush;
  update();
}

const QBrush & QProgressStrip::backgroundColor()
{
  return bgBrush_;
}


void QProgressStrip::setTextColor(const QColor & color)
{
  textColor_ = color;
  update();
}

const QColor & QProgressStrip::textColor()
{
  return textColor_;
}

void QProgressStrip::setText(const QString & text)
{
  text_ = text;
}

const QString & QProgressStrip::text() const
{
  return text_;
}

QSize QProgressStrip::sizeHint() const
{
  const QString s =
      text_.isEmpty() ? " XXXXXX / XXXXXX / XXXXXX/ " :
          text_;

  QFontMetrics fm(Base::font());

  return QSize(std::max(64, fm.horizontalAdvance(s)), std::max(12, fm.height()));
}

QSize QProgressStrip::minimumSizeHint() const
{
  return QSize(64, 12);
}

void QProgressStrip::paintEvent(QPaintEvent * event)
{
  QPainter p(this);
  QRect rc(0, 0, width() - 1, height() - 1);

  if ( bgBrush_.style() != Qt::NoBrush ) {
    p.fillRect(rc, bgBrush_);
  }

  const int n =
      strips_.size();

  if( n > 0 ) {

    std::vector<Strip> sorted_strips =
        strips_;

    std::sort(sorted_strips.begin(), sorted_strips.end(),
        [](const Strip & prev, const Strip & next) {
          return next.value < prev.value;
        });

    const double range =
        maxValue_ - minValue_;

    for( int i = 0; i < n; ++i ) {

      const Strip &strip =
          sorted_strips[i];

      const int w =
          std::min(rc.width(),
              (int) ((strip.value - minValue_) * rc.width() / range));

      if ( w > 0 ) {
        QRect rcs(0, 0, w, rc.height());
        p.fillRect(rcs, strip.brush);
      }
    }
  }

  if( !text_.isEmpty() ) {
    p.drawText(6, (rc.height() + p.fontMetrics().height() / 2) / 2, text_);
  }

  p.drawRect(rc);
}
