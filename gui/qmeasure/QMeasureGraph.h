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

  void setCurrentMeasure(QMeasure * cm);

protected Q_SLOTS:
  void clearGraphs();
  void updateGraphs();

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;
  void updateEnableMeasurements();

protected:
  std::set<QMeasure*> cm_;

  QVBoxLayout *vl_ = nullptr;
  QCustomPlot *plot_ = nullptr;
  QCPGraph *graphs_[4] = { nullptr };
  QCPItemText * textLabel_ = nullptr;
};

class QMeasureGraphDock:
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QMeasureGraphDock ThisClass;
  typedef QCustomDockWidget Base;

  QMeasureGraphDock(const QString & title, QWidget * parent,
      QMeasureGraph * graph);

protected:
  QMeasureSelectionCombo * combobox_ctl = nullptr;
  QToolButton * buttonClear_ctl = nullptr;
};

#endif /* __QMeasureGraph_h__ */
