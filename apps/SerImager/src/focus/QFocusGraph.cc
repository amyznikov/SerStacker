/*
 * QFocusGraph.cc
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#include "QFocusGraph.h"
#include <gui/widgets/style.h>
#include <core/ssprintf.h>

namespace serimager {

#define ICON_menu        ":/qfocus/icons/menu.png"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraph::QFocusGraph(QWidget * parent) :
    Base(parent)
{
  vl_ = new QVBoxLayout(this);
  vl_->addWidget(plot_ = new QCustomPlot(this));


  QPen pen(Qt::red);
  pen.setWidth(2);
  pen.setCosmetic(true);

  for( int i = 0; i < QCameraFocusMeasureThread::MAX_CHANNELS; ++i ) {
    graphs_[i] = plot_->addGraph();
    graphs_[i]->setPen(pen);
  }


  plot_->setBackground(QBrush(QColor(0, 0, 0, 0)));

//  plot_->xAxis2->setVisible(true);
//  plot_->xAxis2->setTickLabels(false);
//  plot_->yAxis2->setVisible(true);
//  plot_->yAxis2->setTickLabels(false);

  QPen axis_pen(QColor(150, 150, 150));
  plot_->xAxis->setBasePen(axis_pen);
  plot_->xAxis->setTickPen(axis_pen);
  plot_->xAxis->setSubTickPen(axis_pen);
  plot_->yAxis->setBasePen(axis_pen);
  plot_->yAxis->setTickPen(axis_pen);
  plot_->yAxis->setSubTickPen(axis_pen);
//  plot_->xAxis2->setBasePen(axis_pen);
//  plot_->xAxis2->setTickPen(axis_pen);
//  plot_->xAxis2->setSubTickPen(axis_pen);
//  plot_->yAxis2->setBasePen(axis_pen);
//  plot_->yAxis2->setTickPen(axis_pen);
//  plot_->yAxis2->setSubTickPen(axis_pen);
  //  plot_->xAxis->setAutoTickCount(4);
  //  plot_->yAxis->setAutoTickCount(5);
  //  plot_->xAxis2->setAutoTickCount(4);
  //  plot_->yAxis2->setAutoTickCount(5);
  plot_->xAxis->setRange(0, 120);
  plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
  plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));

  // make left and bottom axes always transfer their ranges to right and top axes:
  //  connect(plot_->xAxis, SIGNAL(rangeChanged(QCPRange)), plot_->xAxis2, SLOT(setRange(QCPRange)));
  //  connect(plot_->yAxis, SIGNAL(rangeChanged(QCPRange)), plot_->yAxis2, SLOT(setRange(QCPRange)));

  actionsMenu_.addAction(enableFocusTrackAction =
      new QAction("Enable focus track", this));

  enableFocusTrackAction->setCheckable(true);
  enableFocusTrackAction->setChecked(false);

  connect(enableFocusTrackAction, &QAction::triggered,
      [this](bool checked) {
        if ( focusMeasureThread_ ) {
          focusMeasureThread_->setEnabled(checked && isVisible());
        }
      });

}

void QFocusGraph::setFocusMeasureThread(QCameraFocusMeasureThread * thread)
{
  if ( focusMeasureThread_ ) {
    focusMeasureThread_->disconnect(this);
  }

  if ((focusMeasureThread_ = thread)) {

    connect(focusMeasureThread_, &QCameraFocusMeasureThread::dataChanged,
        this, &ThisClass::updateFocusGraph,
        Qt::QueuedConnection);

    focusMeasureThread_->setEnabled(isVisible() && enableFocusTrackAction->isChecked());
  }
}

QCameraFocusMeasureThread * QFocusGraph::focusMeasureThread() const
{
  return focusMeasureThread_;
}

QMenu & QFocusGraph::actionsMenu()
{
  return actionsMenu_;
}

void QFocusGraph::clearFocusGraph()
{
  static const QVector<double> empty_keys;
  static const QVector<double> empty_values;

  for( int i = 0; i < QCameraFocusMeasureThread::MAX_CHANNELS; ++i ) {
    graphs_[i]->setData(empty_keys, empty_values);
  }

  plot_->replot();
}

void QFocusGraph::updateFocusGraph()
{
  if ( focusMeasureThread_ ) {

    QMutexLocker lock(&focusMeasureThread_->mutex());

    QVector<double> keys;

    for( int i = 0; i < QCameraFocusMeasureThread::MAX_CHANNELS; ++i ) {

      const QVector<double> &values =
          focusMeasureThread_->data(i);

      keys.clear();

      for( int j = 0, m = values.size(); j < m; ++j ) {
        keys.append(j);
      }

      graphs_[i]->setData(keys, values, true);
    }

    plot_->yAxis->rescale();


    enum COLORID colorid = focusMeasureThread_->colorid();
    if ( colorid != last_colorid_ )  {
      updatePenColors(last_colorid_ = colorid);
    }

    plot_->replot();
  }
}

void QFocusGraph::updatePenColors(enum COLORID colorid)
{
  CF_DEBUG("colorid=%s", toString(colorid));

  static const QColor *selectedColors = nullptr;

  if( is_bayer_pattern(colorid) ) {
    // extract_bayer_planes() always orders output channels as[ R G1 B G2 ].
    static const QColor colors[QCameraFocusMeasureThread::MAX_CHANNELS] = {
        Qt::red,
        Qt::green,
        Qt::blue,
        Qt::darkGreen,
    };
    selectedColors = colors;
  }
  else {

    switch (colorid) {
      case COLORID_MONO: {
        static const QColor colors[QCameraFocusMeasureThread::MAX_CHANNELS] = {
            Qt::lightGray,
            Qt::lightGray,
            Qt::lightGray,
            Qt::lightGray,
        };
        selectedColors = colors;
      }
        break;

      case COLORID_RGB: {
        static const QColor colors[QCameraFocusMeasureThread::MAX_CHANNELS] = {
            Qt::red,
            Qt::green,
            Qt::blue,
            Qt::yellow,
        };
        selectedColors = colors;
      }
        break;

      case COLORID_BGR: {
        static const QColor colors[QCameraFocusMeasureThread::MAX_CHANNELS] = {
            Qt::blue,
            Qt::red,
            Qt::green,
            Qt::yellow,
        };
        selectedColors = colors;
      }
        break;

      case COLORID_BGRA:
        default: {
        static const QColor colors[QCameraFocusMeasureThread::MAX_CHANNELS] = {
            Qt::blue,
            Qt::red,
            Qt::green,
            Qt::yellow,
        };
        selectedColors = colors;
      }
        break;
    }
  }

  QPen pen;
  pen.setWidth(3);
  pen.setCosmetic(true);

  for( int i = 0; i < QCameraFocusMeasureThread::MAX_CHANNELS; ++i ) {
    pen.setColor(selectedColors[i]);
    graphs_[i]->setPen(pen);
  }
}


void QFocusGraph::showEvent(QShowEvent *event)
{
  Base::showEvent(event);

  if ( focusMeasureThread_ ) {
    focusMeasureThread_->setEnabled(enableFocusTrackAction->isChecked() && isVisible());
  }

}

void QFocusGraph::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  if ( focusMeasureThread_ ) {
    focusMeasureThread_->setEnabled(enableFocusTrackAction->isChecked() && isVisible());
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraphDock::QFocusGraphDock(const QString & title, QWidget * parent, QFocusGraph * view) :
    Base(title, parent, view)
{
  if( view ) {

    menuButton_ =
        Base::titleBar()->addButton(getIcon(ICON_menu),
            "FocusGraph actions...");

    connect(menuButton_, &QToolButton::clicked,
        [this]() {

          QFocusGraph * fg =
              dynamic_cast<QFocusGraph * >(this->widget());

          if ( fg ) {

            QMenu & menu =
                fg->actionsMenu();

            if ( !menu.isEmpty() ) {

              menu.exec(menuButton_->mapToGlobal(
                  QPoint(menuButton_->width()/2,
                      menuButton_->height()/2)));

            }
          }
        });
  }
}


} /* namespace serimager */
