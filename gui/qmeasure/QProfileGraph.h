/*
 * QProfileGraph.h
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QProfileGraph_h__
#define __QProfileGraph_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomplot/qcustomplot.h>

class QProfileGraph :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QProfileGraph ThisClass;
  typedef QWidget Base;

  QProfileGraph(QWidget * parent = nullptr);

Q_SIGNALS:
  void visibilityChanged(bool visble);

protected Q_SLOTS:
  void clearGraphs();
  void updateGraphs();

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

protected:
  QVBoxLayout *vl_ = nullptr;
  QCustomPlot *plot_ = nullptr;
  QCPGraph *graphs_[4] = { nullptr };
  // QCPItemText * textLabel_ = nullptr;
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
  QProfileGraph * profileGraph_ctl = nullptr;
};

#endif /* __QPofileGraph_h__ */
