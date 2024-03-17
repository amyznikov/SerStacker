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

  void showProfilePlot(const QLineF & line, const cv::Mat & image, const cv::Mat & mask);
  void showProfilePlot(const QLine & line, const cv::Mat & image, const cv::Mat & mask);

  const QLine & currentLine() const;

  void setLineStyle(QCPGraph::LineStyle v);
  QCPGraph::LineStyle lineStyle() const;

  void setFixXMin(bool v);
  bool fixXMin() const;

  void setFixXMax(bool v);
  bool fixXMax() const;

  void setFixYMin(bool v);
  bool fixYMin() const;

  void setFixYMax(bool v);
  bool fixYMax() const;

  void setSkipZeroPixels(bool v);
  bool skipZeroPixels() const;

  void setSkipMaskedPixels(bool v);
  bool skipMaskedPixels() const;

  void setXRangeMin(double v);
  double xRangeMin() const;

  void setXRangeMax(double v);
  double xRangeMax() const;

  void setYRangeMin(double v);
  double yRangeMin() const;

  void setYRangeMax(double v);
  double yRangeMax() const;

  void saveParameters(const QString & profileName);
  void loadParameters(const QString & profileName);

Q_SIGNALS:
  void visibilityChanged(bool visble);
  void xRangeRescaled();
  void yRangeRescaled();
  // void parameterChanged();

protected Q_SLOTS:
  void onCopyToClipboardActionTriggered();
  void onShowSettingsActionTriggered(bool checked);
  void onShowStatusbarActionTriggered(bool checked);
  void onCustomPlotMouseMove(QMouseEvent * e);

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;
  void replot();

protected:
  QVBoxLayout * vl_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QStatusBar * statusBar_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * showSettingsAction_ = nullptr;
  QAction * showStatusbarAction_ = nullptr;
  QProfileGraphSettingsDialogBox * plotSettings_ctl = nullptr;

  QCustomPlot *plot_ = nullptr;
  QCPGraph *graphs_[4] = { nullptr };
  QCPGraph::LineStyle lineStyle_ = QCPGraph::lsLine;

  QVector<double> current_keys_;
  QVector<double> current_values_[4];
  QVector<uint8_t> current_ptmasks_;

  QLine currentLine_;
  bool fixXMin_ = false;
  bool fixXMax_ = false;
  bool fixYMin_ = false;
  bool fixYMax_ = false;

  bool skipZeroPixels_ = false;
  bool skipMaskedPixels_ = false;
};


class QProfileGraphSettings :
    public QSettingsWidgetTemplate<QProfileGraph>
{
  Q_OBJECT;
public:
  typedef QProfileGraphSettings ThisClass;
  typedef QSettingsWidgetTemplate<QProfileGraph> Base;

  QProfileGraphSettings(QWidget * parent = nullptr);

  void setProfileGraph(QProfileGraph * profileGraph);
  QProfileGraph * profileGraph() const;

protected:
  void set_options(QProfileGraph * profileGraph) override;
  // void onupdatecontrols() override;
  // void onload(QSettings & settings) override;

protected:
  QEnumComboBox<QCPGraph::LineStyle> * lineStyle_ctl = nullptr;

  QCheckBox * fixXMin_ctl = nullptr;
  QNumericBox * xRangeMin_ctl = nullptr;
  QCheckBox * fixXMax_ctl = nullptr;
  QNumericBox * xRangeMax_ctl = nullptr;

  QCheckBox * fixYMin_ctl = nullptr;
  QNumericBox * yRangeMin_ctl = nullptr;
  QCheckBox * fixYMax_ctl = nullptr;
  QNumericBox * yRangeMax_ctl = nullptr;

  QCheckBox * skipZeroPixels_ctl = nullptr;
  QCheckBox * skipMaskedPixels_ctl = nullptr;
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
