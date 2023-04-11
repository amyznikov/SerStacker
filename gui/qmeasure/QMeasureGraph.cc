/*
 * QMeasureGraph.cc
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#include "QMeasureGraph.h"
#include <gui/widgets/style.h>

//#define ICON_menu         ":/qmeasure/icons/menu.png"
//#define ICON_chart        ":/qmeasure/icons/chart.png"
//#define ICON_roi          ":/qmeasure/icons/roi.png"
//#define ICON_options      ":/qmeasure/icons/options.png"

QMeasureGraph::QMeasureGraph(QWidget * parent) :
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

  if( iconStyleSelector().contains("light", Qt::CaseInsensitive) ) {
    plot_->setBackground(QBrush(QColor(0, 0, 0, 0)));
    plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
    plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));
  }


  static const QColor colors[4] = {
      Qt::blue,
      Qt::red,
      Qt::green,
      Qt::darkYellow,
  };

  pen.setWidth(3);
  for( int i = 0; i < 4; ++i ) {
    pen.setColor(colors[i]);
    graphs_[i]->setPen(pen);
  }

  ///////////////////////////////////////////////////////////////////
}


void QMeasureGraph::setMeasureProvider(QMeasureProvider * provider)
{
  if( mp_ ) {
    mp_->disconnect(this);
  }

  if( (mp_ = provider) ) {

    connect(mp_, &QMeasureProvider::measurementsChanged,
        this, &ThisClass::updateGraphs,
        Qt::QueuedConnection);

    //mp_->setEnabled(isVisible() && enableTrackAction_->isChecked());
  }

//  if( settings_ctl ) {
//    settings_ctl->setFocusMeasureProvider(provider_);
//  }

}

QMeasureProvider* QMeasureGraph::measureProvider() const
{
  return mp_;
}

void QMeasureGraph::clearGraphs()
{
  static const QVector<double> empty_keys;
  static const QVector<double> empty_values;

  for( int i = 0; i < 4; ++i ) {
    graphs_[i]->setData(empty_keys, empty_values);
  }

  plot_->replot();
}

void QMeasureGraph::updateGraphs()
{
  using Frame = QMeasureProvider::MeasuredFrame;

  if( mp_ ) {

    const QMeasure *selected_measure =
        mp_->selected_measures().empty() ? nullptr :
            *mp_->selected_measures().begin();

    if ( selected_measure ) {

      QVector<double> keys[4];
      QVector<double> values[4];
      int max_key = 0;
      int i = 0;

      for( auto ii = mp_->measured_frames().begin(); ii != mp_->measured_frames().end(); ++i, ++ii ) {
        for( const auto &m : ii->measurements ) {
          if ( m.measure == selected_measure && m.cn > 0 ) {
            for ( int j = 0; j < m.cn; ++j ) {
              keys[j].append(i);
              values[j].append(m.value(j));
              if ( i > max_key ) {
                max_key = i;
              }
            }
          }
        }
      }

      for( i = 0; i < 4; ++i ) {
        graphs_[i]->setData(keys[i], values[i], true);
      }

      plot_->yAxis->rescale();
      plot_->xAxis->setRange(0, 1.2 * max_key);

      //      enum COLORID colorid = provider_->colorid();
      //      if( colorid != last_colorid_ ) {
      //        updatePenColors(last_colorid_ = colorid);
      //      }

      plot_->replot();
    }
  }
}

void QMeasureGraph::showEvent(QShowEvent * event)
{
  Base::showEvent(event);
}

void QMeasureGraph::hideEvent(QHideEvent * event)
{
  Base::hideEvent(event);
}

// void QMeasureGraph::updatePenColors()
// {
// }


QMeasureGraphDock::QMeasureGraphDock(const QString & title, QWidget * parent, QMeasureGraph * view) :
    Base(title, parent, view)
{
//  if( view ) {
//
//    const QList<QAction*> actions =
//        view->actions();
//
//    QCustomDockTitleBar *bar =
//        titleBar();
//
//    for( QAction *action : actions ) {
//      QToolButton *tb =
//          bar->addButton(action);
//    }
//
//  }
}
