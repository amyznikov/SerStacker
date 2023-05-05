/*
 * QProfileGraph.cc
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#include "QProfileGraph.h"
#include <gui/widgets/style.h>


#define ICON_plot      ":/qmeasure/icons/plot.png"


QProfileGraph::QProfileGraph(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qmeasure_resources);

  vl_ = new QVBoxLayout(this);
  vl_->addWidget(plot_ = new QCustomPlot(this));

  QPen pen(Qt::red);
  pen.setWidth(2);
  pen.setCosmetic(true);

  for( int i = 0; i < 4; ++i ) {
    graphs_[i] = plot_->addGraph();
    graphs_[i]->setPen(pen);
  }

  QPen axis_pen(QColor(150, 150, 150));
  plot_->xAxis->setBasePen(axis_pen);
  plot_->xAxis->setTickPen(axis_pen);
  plot_->xAxis->setSubTickPen(axis_pen);
  plot_->yAxis->setBasePen(axis_pen);
  plot_->yAxis->setTickPen(axis_pen);
  plot_->yAxis->setSubTickPen(axis_pen);
  plot_->xAxis->setRange(0, 120);

  static const QColor colors[4] = {
      Qt::blue,
      Qt::green,
      Qt::red,
      Qt::darkYellow,
  };

  pen.setWidth(3);
  for( int i = 0; i < 4; ++i ) {
    pen.setColor(colors[i]);
    graphs_[i]->setPen(pen);
  }

  ///////////////////////////////////////////////////////////////////

  // add the text label at the top:

  if( iconStyleSelector().contains("light", Qt::CaseInsensitive) ) {
    plot_->setBackground(QBrush(QColor(0, 0, 0, 0)));
    plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
    plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));
  }
}


void QProfileGraph::showEvent(QShowEvent * event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QProfileGraph::hideEvent(QHideEvent * event)
{
  Base::hideEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QProfileGraph::clearGraphs()
{

}

void QProfileGraph::updateGraphs()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphDialogBox::QProfileGraphDialogBox(QWidget * parent) :
    ThisClass("Plot Image Profile", parent)
{
}

QProfileGraphDialogBox::QProfileGraphDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * lv =
      new QVBoxLayout(this);

  lv->setContentsMargins(0,0,0,0);

  lv->addWidget(profileGraph_ctl =
      new QProfileGraph(this));
}

QProfileGraph * QProfileGraphDialogBox::profileGraph() const
{
  return profileGraph_ctl;
}

void QProfileGraphDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QProfileGraphDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

