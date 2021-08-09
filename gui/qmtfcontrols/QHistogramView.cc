/*
 * QHistogramView.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QHistogramView.h"
#include <core/debug.h>

static constexpr int MARGIN = 2;

QHistogramView::QHistogramView(QWidget * parent)
  : Base(parent)
{
  setMinimumSize(2 * MARGIN, 2 * MARGIN);
  resize(256, (int)(256 / 1.62));
}

void QHistogramView::setLogScale(bool v)
{
  logScale_ = v;
  update_scaled_counts();
}

bool QHistogramView::logScale() const
{
  return logScale_;
}


void QHistogramView::setBackgroundColor(const QColor & v)
{
  backgroundColor_ = v;
}

const QColor & QHistogramView::backgroundColor() const
{
  return backgroundColor_ ;
}

void QHistogramView::setForegroundColor(const QColor & v)
{
  foregroundColor_ = v;
}

const QColor & QHistogramView::foregroundColor() const
{
  return foregroundColor_;
}

void QHistogramView::setChartType(ChartType v)
{
  chartType_ = v;
  if ( isVisible() ) {
    update();
  }
}

QHistogramView::ChartType QHistogramView::chartType() const
{
  return chartType_;
}


void QHistogramView::showHistogram(const cv::Mat1f & histogram, int channel,
    int first_bin, float first_level,
    int last_bin, float last_level)
{
  histogram.copyTo(this->histogram_);
  this->channel_ = channel;
  this->first_bin_ = std::max(0, first_bin);
  this->last_bin_ = std::min(last_bin, histogram_.rows - 1);
  this->first_level_ = first_level;
  this->last_level_ = last_level;
  update_scaled_counts();
  update();
}

double QHistogramView::scaled_count(double c) const
{
  return c > 0 ? (logScale_ ? log(1 + c) : c) : 0;
}

void QHistogramView::update_scaled_counts()
{
  scaled_counts_.clear();
  if ( histogram_.rows > 0 ) {

    scaled_counts_.resize(histogram_.rows);
    double cmax = 0;

    for ( int i = 0; i < histogram_.rows; ++i ) {

      if ( channel_ >= 0 && channel_ < histogram_.cols ) {
        scaled_counts_[i] = scaled_count(histogram_[i][channel_]);
      }
      else {
        double sum = 0;
        for ( int j = 0; j < histogram_.cols; ++j ) {
          sum += histogram_[i][j];
        }
        scaled_counts_[i] = scaled_count(sum);
      }

      if ( scaled_counts_[i] > cmax ) {
        cmax = scaled_counts_[i];
      }
    }

    cv::multiply(scaled_counts_, 1. / cmax, scaled_counts_);
  }
}


QSize QHistogramView::sizeHint() const
{
  const int width = std::max(256, this->screen()->geometry().width() / 3);
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
  const QSize size = this->size();
  const int l = MARGIN;
  const int t = MARGIN;
  const int r = size.width() - MARGIN - 1;
  const int b = size.height() - MARGIN - 1;
  const int w = r - l;
  const int h = b - t;
  const int bins = last_bin_ - first_bin_ + 1;

  QPen pen;
  pen.setColor(foregroundColor_);
  pen.setWidth(std::max(5., ceil((double) w / bins)));
  p.setPen(pen);


  int prev_xpix = l;
  double cmax = scaled_counts_[first_bin_];
  for ( int i = first_bin_ + 1; i <= last_bin_; ++i ) {

    const int xpix = (l + (i - first_bin_) * w / bins);
    if ( xpix == prev_xpix ) {
      if ( scaled_counts_[i] > cmax ) {
        cmax = scaled_counts_[i];
      }
      if ( i < last_bin_ ) {
        continue;
      }
    }

    if ( cmax > 0 ) {
      const int ypix = (int)(b - cmax * h);
      p.drawLine(prev_xpix, b, prev_xpix, ypix);
    }

    cmax = scaled_counts_[i];
    prev_xpix = xpix;
  }

}

void QHistogramView::drawLineChart(QPainter & p) const
{
  const QSize size = this->size();
  const int l = MARGIN;
  const int t = MARGIN;
  const int r = size.width() - MARGIN - 1;
  const int b = size.height() - MARGIN - 1;
  const int w = r - l;
  const int h = b - t;
  const int bins = last_bin_ - first_bin_ + 1;

  QPen pen;
  pen.setColor(Qt::darkGray);
  pen.setWidth(std::max(2., ceil((double) w / bins)));
  p.setPen(pen);


  double cmax = scaled_counts_[first_bin_];
  int prev_xpix = l;
  int prev_ypix = (int)(b - cmax * h);
  for ( int i = first_bin_ + 1; i <= last_bin_; ++i ) {

    const int xpix = (l + (i - first_bin_) * w / bins);
    if ( xpix == prev_xpix ) {
      if ( scaled_counts_[i] > cmax ) {
        cmax = scaled_counts_[i];
      }
      if ( i < last_bin_ ) {
        continue;
      }
    }

    const int ypix = (int)(b - cmax * h);
    if ( cmax > 0 ) {
      p.drawLine(prev_xpix, prev_ypix, prev_xpix, ypix);
    }

    cmax = scaled_counts_[i];
    prev_xpix = xpix;
    prev_ypix = ypix;
  }
}

void QHistogramView::paintEvent(QPaintEvent *event)
{
  const QSize size = this->size();

  QPainter p(this);

  p.fillRect(0, 0, size.width(), size.height(),
      backgroundColor_);

  if ( size.width() <= 2 * MARGIN || size.height() <= 2 * MARGIN ) {
    return;  // too small wiindow
  }

  if (  scaled_counts_.empty() ) {
    return; // nothing to draw
  }

  switch (chartType_) {
    case ChartType_Lines:
      drawLineChart(p);
      break;
    default:
      drawBarChart(p);
      break;
  }

  drawMajorGrid(p);
  drawBorder(p);
}

void QHistogramView::clear()
{
  histogram_.release();
  update();
}
