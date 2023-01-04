/*
 * QFocusGraph.h
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFocusGraph_h__
#define __QFocusGraph_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomdock/QCustomDock.h>
#include <gui/qcustomplot/qcustomplot.h>
#include "QCameraFocusMeasureThread.h"

namespace serimager {

class QFocusGraph :
    public QWidget
{
  Q_OBJECT;

public:
  typedef QFocusGraph ThisClass;
  typedef QWidget Base;

  QFocusGraph(QWidget * parent = nullptr);

  void setFocusMeasureThread(QCameraFocusMeasureThread * thread);
  QCameraFocusMeasureThread * focusMeasureThread() const;

  QMenu & actionsMenu();

protected Q_SLOTS:
  void clearFocusGraph();
  void updateFocusGraph();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void updatePenColors(enum COLORID colorid);

protected:
  QCameraFocusMeasureThread * focusMeasureThread_ = nullptr;

  QVBoxLayout * vl_ = nullptr;
  QCustomPlot * plot_ = nullptr;
  QCPGraph *graphs_[QCameraFocusMeasureThread::MAX_CHANNELS] = { nullptr };
  enum COLORID last_colorid_ = COLORID_UNKNOWN;

  QMenu actionsMenu_;
  QAction * enableFocusTrackAction = nullptr;
};


class QFocusGraphDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QFocusGraphDock ThisClass;
  typedef QCustomDockWidget Base;

  QFocusGraphDock(const QString & title, QWidget * parent, QFocusGraph * view);

protected:
  QToolButton *menuButton_ = nullptr;

};


} /* namespace serimager */

#endif /* __QFocusGraph_h__ */
