/*
 * QProfileGraph.h
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QProfileGraph_h__
#define __QProfileGraph_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qcustomplot/qcustomplot.h>
#include <opencv2/opencv.hpp>

class QProfileGraphSettingsDialogBox;

class QProfileGraph :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QProfileGraph ThisClass;
  typedef QWidget Base;

  QProfileGraph(QWidget * parent = nullptr);

  void showProfilePlot(const QLineF & line, const cv::Mat & image);
  void showProfilePlot(const QLine & line, const cv::Mat & image);

  const QLine & currentLine() const;

  void setLineStyle(QCPGraph::LineStyle v);
  QCPGraph::LineStyle lineStyle() const;

  void setFixXRange(bool v);
  bool fixXRange() const;

  void setFixYRange(bool v);
  bool fixYRange() const;

  void setSkipZeroPixels(bool v);
  bool skipZeroPixels() const;

  void setXRangeMin(double v);
  double xRangeMin() const;

  void setXRangeMax(double v);
  double xRangeMax() const;

  void setYRangeMin(double v);
  double yRangeMin() const;

  void setYRangeMax(double v);
  double yRangeMax() const;


  void replot();

Q_SIGNALS:
  void visibilityChanged(bool visble);
  void xRangeRescaled();
  void yRangeRescaled();
  void skipZeroPixlelsChanged();

protected Q_SLOTS:
  void onShowSettingsActionTriggered(bool checked);
  void onCopyToClipboardActionTriggered();

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

protected:
  QVBoxLayout * vl_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * showSettingsAction_ = nullptr;
  QProfileGraphSettingsDialogBox * plotSettings_ctl = nullptr;

  QCustomPlot *plot_ = nullptr;
  QCPGraph *graphs_[4] = { nullptr };
  QCPGraph::LineStyle lineStyle_ = QCPGraph::lsLine;

  QVector<double> current_keys_[4];
  QVector<double> current_values_[4];

  QLine currentLine_;
  bool fixXRange_ = false;
  bool fixYRange_ = false;

  bool skipZeroPixels_ = false;
};


class QProfileGraphSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QProfileGraphSettings ThisClass;
  typedef QSettingsWidget Base;

  QProfileGraphSettings(QWidget * parent = nullptr);

  void setProfileGraph(QProfileGraph * profileGraph);
  QProfileGraph * profileGraph() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  QProfileGraph * profileGraph_ = nullptr;

  QEnumComboBox<QCPGraph::LineStyle> * lineStyle_ctl = nullptr;

  QCheckBox * fixXRange_ctl = nullptr;
  QNumericBox * xRangeMin_ctl = nullptr;
  QNumericBox * xRangeMax_ctl = nullptr;

  QCheckBox * fixYRange_ctl = nullptr;
  QNumericBox * yRangeMin_ctl = nullptr;
  QNumericBox * yRangeMax_ctl = nullptr;

  QCheckBox * skipZeros_ctl = nullptr;
};


class QProfileGraphDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QProfileGraphDialogBox ThisClass;
  typedef QDialog Base;

  QProfileGraphDialogBox(QWidget * parent = nullptr);
  QProfileGraphDialogBox(const QString & title, QWidget * parent = nullptr);

  QProfileGraph * profileGraph() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QProfileGraph *profileGraph_ctl = nullptr;
};

class QProfileGraphSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QProfileGraphSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QProfileGraphSettingsDialogBox(QWidget * parent);
  QProfileGraphSettingsDialogBox(const QString & title, QWidget * parent);

  void setProfileGraph(QProfileGraph * profileGraph);
  QProfileGraph * profileGraph() const;

  QProfileGraphSettings * settingsWidget() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QProfileGraphSettings * settingsWidget_ = nullptr;
};

#endif /* __QPofileGraph_h__ */
