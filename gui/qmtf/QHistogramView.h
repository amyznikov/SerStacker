/*
 * QHistogramView.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QHistogramView_h__
#define __QHistogramView_h__

#include <QtWidgets/QtWidgets>
#include <opencv2/opencv.hpp>

class QHistogramView
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QHistogramView ThisClass;
  typedef QWidget Base;

  enum ChartType {
    ChartType_Lines,
    ChartType_Bars,
  };

  enum DisplayChannel {
    DisplayChannel_Value,
    DisplayChannel_Grayscale,
    DisplayChannel_RGB,
  };

  QHistogramView(QWidget * parent = Q_NULLPTR);

  void setHistogram(cv::InputArray H, double hmin, double hmax);
  void setMtfCurve(std::vector<float> && cy);

  void setLogScale(bool v);
  bool logScale() const;

  void setBackgroundColor(const QColor & v);
  const QColor & backgroundColor() const;

  void setForegroundColor(const QColor & v);
  const QColor & foregroundColor() const;

  void setChartType(ChartType v);
  ChartType chartType() const;

  void setDisplayChannel(DisplayChannel v);
  DisplayChannel displayChannel() const;

  void setSizeHint(const QSize & s);
  QSize sizeHint() const override;

  void clear();


protected:
  void paintEvent(QPaintEvent *event) override;
  void drawMajorGrid(QPainter & p) const;
  void drawBorder(QPainter & p) const;
  void drawBarChart(QPainter & p) const;
  void drawLineChart(QPainter & p) const;
  void drawMtfCurve(QPainter & p) const;

protected:

protected:
  QSize _sizeHint;
  QColor _backgroundColor = Qt::white;
  QColor _foregroundColor = Qt::lightGray;
  ChartType _chartType = ChartType_Lines;
  DisplayChannel _displayChannel = DisplayChannel_RGB;

  std::vector<float> _mtfCurve;
  cv::Mat1f H, LH;
  double hmin = -1, hmax = -1;
  bool _logScale = true;
};

#endif /* __QHistogramView_h__ */
