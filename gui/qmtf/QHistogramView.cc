/*
 * QHistogramView.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QHistogramView.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/histogram.h>
#include <core/debug.h>

static constexpr int MARGIN = 2;

static QColor defaultChannelColors [] = {
    QColor(Qt::blue),
    QColor(Qt::green),
    QColor(Qt::red),
    QColor(Qt::darkGray),
};


QHistogramView::QHistogramView(QWidget * parent)
  : Base(parent)
{
  setMinimumSize( 64 + 2 * MARGIN, 64 + 2 * MARGIN);
  resize(128, (int)(128 / 1.62));
}

void QHistogramView::setLogScale(bool v)
{
  if ( _logScale != v ) {
    _logScale = v;
    if( isVisible() ) {
      //repaint();
      update();
    }
  }
}

bool QHistogramView::logScale() const
{
  return _logScale;
}


void QHistogramView::setBackgroundColor(const QColor & v)
{
  if ( _backgroundColor != v ) {
    _backgroundColor = v;
    if( isVisible() ) {
      //repaint();
      update();
    }
  }
}

const QColor & QHistogramView::backgroundColor() const
{
  return _backgroundColor ;
}

void QHistogramView::setForegroundColor(const QColor & v)
{
  if ( _foregroundColor != v ) {
    _foregroundColor = v;
    if( isVisible() ) {
      //repaint();
      update();
    }
  }
}

const QColor & QHistogramView::foregroundColor() const
{
  return _foregroundColor;
}

void QHistogramView::setChartType(ChartType v)
{
  if ( _chartType != v ) {
    _chartType = v;
    if ( isVisible() ) {
      //repaint();
      update();
    }
  }
}

QHistogramView::ChartType QHistogramView::chartType() const
{
  return _chartType;
}

void QHistogramView::setDisplayChannel(DisplayChannel v)
{
  if ( _displayChannel != v ) {
    _displayChannel = v;
    if ( isVisible() ) {
      //repaint();
      update();
    }
  }
}

QHistogramView::DisplayChannel QHistogramView::displayChannel() const
{
  return _displayChannel;
}

void QHistogramView::setHistogram(cv::InputArray _H, double hmin, double hmax)
{
  double min = 0, max = 1;

  this->hmin = hmin;
  this->hmax = hmax;

  if ( _H.getMat().data != this->H.data ) {
    _H.copyTo(this->H);
  }

  if ( !this->H.empty() ) {
    cv::log(H + 1, LH);

    cv::minMaxLoc(H, &min, &max);
    if( max > 0 ) {
      cv::multiply(H, 0.8 / max, H);
    }

    cv::minMaxLoc(LH, &min, &max);
    if( max > 0 ) {
      cv::multiply(LH, 0.8 / max, LH);
    }
  }

  if( isVisible() ) {
    // repaint();
    update();
  }
}

void QHistogramView::setMtfCurve(std::vector<float> && cy)
{
  _cy = cy;
  if( isVisible() ) {
    // repaint();
    update();
  }
}

void QHistogramView::setSizeHint(const QSize & s)
{
  _sizeHint = s;
}

QSize QHistogramView::sizeHint() const
{
  if ( !_sizeHint.isEmpty() ) {
    return _sizeHint;
  }

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  const int width = std::max(256, this->screen()->geometry().width() / 3);
#else
  const int width = std::max(256, QGuiApplication::primaryScreen()->geometry().width() / 3);
#endif

  return QSize(width, (int) (width / 2));
}

void QHistogramView::drawMajorGrid(QPainter & p) const
{
  constexpr int xparts = 8;
  constexpr int yparts = 8;

  const QSize size = this->size();
  const int w = size.width() - 2 * MARGIN;
  const int h = size.height() - 2 * MARGIN;

  int xpix, ypix;

  p.setPen(Qt::darkGray);
  for ( int i = 1; i < xparts; ++i ) {
    if ( i != xparts / 2 ) {
      xpix = MARGIN + i * w / xparts;
      p.drawLine(xpix, MARGIN, xpix, size.height() - MARGIN - 1);
    }
  }

  for ( int i = 1; i < yparts; ++i ) {
    if ( i != yparts / 2 ) {
      ypix = MARGIN + i * h / yparts;
      p.drawLine(MARGIN, ypix, size.width() - MARGIN - 1, ypix);
    }
  }

  p.setPen(Qt::black);
  xpix = MARGIN + w / 2;
  p.drawLine(xpix, MARGIN, xpix, size.height() - MARGIN - 1);

  ypix = MARGIN + h / 2;
  p.drawLine(MARGIN, ypix, size.width() - MARGIN - 1, ypix);
}

void QHistogramView::drawBorder(QPainter & p) const
{
  const QSize size = this->size();
  p.setPen(Qt::black);
  p.drawRect(0, 0, size.width()-1, size.height()-1);
}


void QHistogramView::drawBarChart(QPainter & p) const
{
  if ( H.rows < 1 ) {
    return;
  }

  const QSize size = this->size();
  const int l = MARGIN;
  const int t = MARGIN;
  const int r = size.width() - MARGIN - 1;
  const int b = size.height() - MARGIN - 1;
  const int w = r - l;
  const int h = b - t;

  const cv::Mat1f levels =
      _logScale ? LH : H;

  for ( int c = 0; c < levels.cols; ++c ) {

    QPen pen;
    pen.setColor(defaultChannelColors[c % 4]);
    pen.setWidth(std::max(3., ceil((double) w / H.rows)));
    p.setPen(pen);

    int prev_xpix = l;
    double cmax = levels[0][c];

    for ( int r = 1; r < levels.rows ; ++r ) {

      const int xpix = (l + r ) * w / levels.rows;
      if ( xpix == prev_xpix ) {
        if ( levels[r][c] > cmax ) {
          cmax = levels[r][c];
        }
        if ( r < levels.rows - 1 ) {
          continue;
        }
      }

      if ( cmax > 0 ) {
        const int ypix = (int) (b - cmax * h);
        p.drawLine(prev_xpix, b, prev_xpix, ypix);
      }

      cmax = levels[r][c];
      prev_xpix = xpix;
    }

  }

}

void QHistogramView::drawLineChart(QPainter & p) const
{
  if ( H.rows < 1 ) {
    return;
  }

  const QSize size = this->size();
  const int l = MARGIN;
  const int t = MARGIN;
  const int r = size.width() - MARGIN - 1;
  const int b = size.height() - MARGIN - 1;
  const int w = r - l;
  const int h = b - t;


  const cv::Mat1f levels =
      _logScale ? LH : H;

  for ( int c = 0; c < levels.cols; ++c ) {

    QPen pen;
    pen.setColor(defaultChannelColors[c % 4]);
    pen.setWidth(2);
    p.setPen(pen);

    int prev_xpix = l;
    int prev_ypix = (int)(b - levels[0][c] * h);

    for ( int r = 1; r < levels.rows; ++r ) {

      const int xpix = (int)(l + r ) * w / levels.rows;
      const int ypix = (int)(b - levels[r][c] * h);

      if ( xpix != prev_xpix || ypix != prev_ypix ) {
        p.drawLine(prev_xpix, prev_ypix, xpix, ypix);
        prev_xpix = xpix;
        prev_ypix = ypix;
      }
    }
  }
}

void QHistogramView::drawMtfCurve(QPainter & p) const
{
  if ( !_cy.empty() ) {

    QPainterPath path;

    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(2);
    p.setPen(pen);

    const QSize size = this->size();
    const int l = MARGIN;
    const int t = MARGIN;
    const int r = size.width() - MARGIN - 1;
    const int b = size.height() - MARGIN - 1;
    const int w = r - l;
    const int h = b - t;

    int prev_xpix = l;
    int prev_ypix = (int)(b - _cy[0] * h);

    for ( size_t i = 0, n = _cy.size(); i < n; ++i ) {
      const float x = (float)(i) / n;
      const float y = _cy[i];

      const int xpix = (int) (l + x * w);
      const int ypix = (int) (b - y * h);

      if ( xpix != prev_xpix || ypix != prev_ypix ) {
        path.moveTo(prev_xpix, prev_ypix);
        path.lineTo(xpix, ypix);
        prev_xpix = xpix;
        prev_ypix = ypix;
      }
    }

    p.drawPath(path);
  }
}

void QHistogramView::paintEvent(QPaintEvent *event)
{
  const QSize size = this->size();

  QPainter p(this);

  p.fillRect(0, 0, size.width(), size.height(),
      _backgroundColor); // palette().window().color()/**/

  if ( size.width() <= 2 * MARGIN || size.height() <= 2 * MARGIN ) {
    return;  // too small wiindow
  }

  switch ( _chartType ) {
  case ChartType_Lines :
    drawLineChart(p);
    break;
  default :
    drawBarChart(p);
    break;
  }

  drawMtfCurve(p);
  drawMajorGrid(p);
  drawBorder(p);
}

void QHistogramView::clear()
{
  H.release();
  LH.release();
  //repaint();
  update();
}
