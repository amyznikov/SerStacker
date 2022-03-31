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

  void setImage(cv::InputArray image, cv::InputArray mask = cv::noArray());
  void setHistogram(cv::InputArray H, double hmin, double hmax);

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

  void clear();


protected:
  QSize sizeHint() const override;
  void paintEvent(QPaintEvent *event) override;
  void drawMajorGrid(QPainter & p) const;
  void drawBorder(QPainter & p) const;
  void drawBarChart(QPainter & p) const;
  void drawLineChart(QPainter & p) const;

protected:
  void updateHistogram();

protected:
  QColor backgroundColor_ = Qt::white;
  QColor foregroundColor_ = Qt::lightGray;
  ChartType chartType_ = ChartType_Lines;
  DisplayChannel displayChannel_ = DisplayChannel_RGB;

  cv::Mat image_, mask_;
  cv::Mat1f H, LH;
  double hmin = -1, hmax = -1;
  bool logScale_ = true;
};

#endif /* __QHistogramView_h__ */
