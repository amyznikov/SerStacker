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

class QHistogramView: public QWidget
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


//  void showHistogram(const cv::Mat1f & histogram, int channel,
//      int first_bin, float first_level,
//      int last_bin, float last_level);

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
  //  double scaled_count(double c) const;
  //  void update_scaled_counts();

protected:
  QColor backgroundColor_ = Qt::white;
  QColor foregroundColor_ = Qt::lightGray;
  ChartType chartType_ = ChartType_Lines;
  DisplayChannel displayChannel_ = DisplayChannel_RGB;

  cv::Mat image_, mask_;
  cv::Mat1f H, LH;
  double hmin = -1, hmax = -1;
//  cv::Mat1f histogram_;
//  std::vector<double> scaled_counts_;
//  int channel_ = 0;
//  int first_bin_ = 0;
//  float first_level_ = 0;
//  int last_bin_ = 0;
//  float last_level_ = 0;
  bool logScale_ = true;
};

#endif /* __QHistogramView_h__ */
