/*
 * QMeasureGraph.h
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureGraph_h__
#define __QMeasureGraph_h__

#include <gui/qcustomdock/QCustomDock.h>
#include <gui/qcustomplot/qcustomplot.h>
#include "QMeasureProvider.h"
#include "QMeasureSelection.h"

class QMeasureGraph :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QMeasureGraph ThisClass;
  typedef QWidget Base;

  QMeasureGraph(QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * provider);
  QMeasureProvider* measureProvider() const;

  void setCurrentMeasure(QMeasure * cm);
  QMeasure * currentMeasure() const;

protected Q_SLOTS:
  void clearGraphs();
  void updateGraphs();

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;
  //  void updatePenColors();

protected:
  QMeasureProvider* mp_ = nullptr;
  QMeasure * cm_ = nullptr;

  QVBoxLayout *vl_ = nullptr;
  QCustomPlot *plot_ = nullptr;
  QCPGraph *graphs_[4] = { nullptr };

  //  QAction *enableTrackAction_ = nullptr;
  //  QAction *showSettingsAction_ = nullptr;
  // QFocusGraphSettingsDialogBox *settings_ctl = nullptr;

};

class QMeasureGraphDock:
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QMeasureGraphDock ThisClass;
  typedef QCustomDockWidget Base;

  QMeasureGraphDock(const QString & title, QWidget * parent,
      QMeasureGraph * view);
};

#endif /* __QMeasureGraph_h__ */
