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

#define ICON_menu         ":/qfocus/icons/menu.png"
#define ICON_chart        ":/qfocus/icons/chart.png"
#define ICON_roi          ":/qfocus/icons/roi.png"
#define ICON_options      ":/qfocus/icons/options.png"


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

  QPen axis_pen(QColor(150, 150, 150));
  plot_->xAxis->setBasePen(axis_pen);
  plot_->xAxis->setTickPen(axis_pen);
  plot_->xAxis->setSubTickPen(axis_pen);
  plot_->yAxis->setBasePen(axis_pen);
  plot_->yAxis->setTickPen(axis_pen);
  plot_->yAxis->setSubTickPen(axis_pen);
  plot_->xAxis->setRange(0, 120);
  plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
  plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));

  ///////////////////////////////////////////////////////////////////

  addAction(showSettingsAction_ =
      new QAction(getIcon(ICON_options),
          "Options...",
          this));

  showSettingsAction_->setCheckable(true);
  showSettingsAction_->setChecked(false);

  connect(showSettingsAction_, &QAction::triggered,
      [this](bool checked) {
        if ( settings_ctl ) {
          settings_ctl->setVisible(checked);
        }
        else {

          settings_ctl = new QFocusGraphSettingsDialogBox (this);
          settings_ctl->setFocusMeasureThread(focusMeasureThread_);

          connect(settings_ctl, &QFocusGraphSettingsDialogBox::visibilityChanged,
              showSettingsAction_, &QAction::setChecked);

          settings_ctl->show();
        }

      });

  ///////////////////////////////////////////////////////////////////

  addAction(enableFocusTrackAction_ =
      new QAction(getIcon(ICON_chart),
          "Enable focus track",
          this));

  enableFocusTrackAction_->setCheckable(true);
  enableFocusTrackAction_->setChecked(false);
  connect(enableFocusTrackAction_, &QAction::triggered,
      [this](bool checked) {
        if ( focusMeasureThread_ ) {
          focusMeasureThread_->setEnabled(checked && isVisible());
        }
      });

  ///////////////////////////////////////////////////////////////////

  addAction(showRoiAction_ =
      new QAction(getIcon(ICON_roi),
          "Show / Hide ROI...",
          this));

  showRoiAction_->setCheckable(true);
  showRoiAction_->setChecked(false);

  connect(showRoiAction_, &QAction::triggered,
      [this](bool checked) {
        if ( focusMeasureThread_ ) {
        }
      });


  ///////////////////////////////////////////////////////////////////
}

void QFocusGraph::setFocusMeasureThread(QCameraFocusMeasureThread * thread)
{
  if( focusMeasureThread_ ) {
    focusMeasureThread_->disconnect(this);
  }

  if( (focusMeasureThread_ = thread) ) {

    connect(focusMeasureThread_, &QCameraFocusMeasureThread::dataChanged,
        this, &ThisClass::updateFocusGraph,
        Qt::QueuedConnection);

    focusMeasureThread_->setEnabled(isVisible() && enableFocusTrackAction_->isChecked());
  }

  if( settings_ctl ) {
    settings_ctl->setFocusMeasureThread(focusMeasureThread_);
  }
}

QCameraFocusMeasureThread * QFocusGraph::focusMeasureThread() const
{
  return focusMeasureThread_;
}

QAction * QFocusGraph::showRoiAction() const
{
  return showRoiAction_;
}

//QMenu & QFocusGraph::actionsMenu()
//{
//  return actionsMenu_;
//}

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
          focusMeasureThread_->measurements(i);

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
    focusMeasureThread_->setEnabled(enableFocusTrackAction_->isChecked() && isVisible());
  }

}

void QFocusGraph::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  if ( focusMeasureThread_ ) {
    focusMeasureThread_->setEnabled(enableFocusTrackAction_->isChecked() && isVisible());
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraphDock::QFocusGraphDock(const QString & title, QWidget * parent, QFocusGraph * view) :
    Base(title, parent, view)
{
  if( view ) {

    const QList<QAction*> actions =
        view->actions();

    QCustomDockTitleBar * bar =
        titleBar();

    for( QAction *action : actions ) {
      QToolButton *tb =
          bar->addButton(action);
    }



//    menuButton_ =
//        Base::titleBar()->addButton(getIcon(ICON_menu),
//            "FocusGraph actions...");
//
//    connect(menuButton_, &QToolButton::clicked,
//        [this]() {
//
//          QFocusGraph * fg =
//              dynamic_cast<QFocusGraph * >(this->widget());
//
//          if ( fg ) {
//
//            QMenu & menu =
//                fg->actionsMenu();
//
//            if ( !menu.isEmpty() ) {
//
//              menu.exec(menuButton_->mapToGlobal(
//                  QPoint(menuButton_->width()/2,
//                      menuButton_->height()/2)));
//
//            }
//          }
//        });
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraphSettingsWidget::QFocusGraphSettingsWidget(QWidget * parent) :
    Base("QFocusGraphSettings", parent)
{
  eps_ctl =
      add_numeric_box<double>(
          "eps",
          [this](double v) {
            if ( focusMeasureThread_ ) {
              focusMeasureThread_->measure().set_eps(v);
            }
          },
          [this](double * v) {
            if ( focusMeasureThread_ ) {
              *v = focusMeasureThread_->measure().eps();
              return true;
            }
            return false;
          });

  dscale_ctl =
      add_numeric_box<int>(
          "dscale",
          [this](int v) {
            if ( focusMeasureThread_ ) {
              focusMeasureThread_->measure().set_dscale(v);
            }
          },
          [this](int * v) {
            if ( focusMeasureThread_ ) {
              *v = focusMeasureThread_->measure().dscale();
              return true;
            }
            return false;
          });

  equalize_hist_ctl =
      add_checkbox(
          "equalize_hist",
          [this](bool checked ) {
            if ( focusMeasureThread_ ) {
              focusMeasureThread_->measure().set_equalize_hist(checked);
            }
          },
          [this](bool * v) {
            if ( focusMeasureThread_ ) {
              *v = focusMeasureThread_->measure().equalize_hist();
              return true;
            }
            return false;
          });


  updateControls();
}

void QFocusGraphSettingsWidget::setFocusMeasureThread(QCameraFocusMeasureThread * thread)
{
  focusMeasureThread_ = thread;
  updateControls();
}

QCameraFocusMeasureThread * QFocusGraphSettingsWidget::focusMeasureThread() const
{
  return focusMeasureThread_;
}

void QFocusGraphSettingsWidget::onupdatecontrols()
{
  Base::onupdatecontrols();

  if ( !focusMeasureThread_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraphSettingsDialogBox::QFocusGraphSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("QFocusGraphSettings");

  lv_ = new QVBoxLayout(this);
  lv_->addWidget(settings_ctl = new QFocusGraphSettingsWidget(this));
}

void QFocusGraphSettingsDialogBox::setFocusMeasureThread(QCameraFocusMeasureThread * thread)
{
  settings_ctl->setFocusMeasureThread(thread);
}

QCameraFocusMeasureThread * QFocusGraphSettingsDialogBox::focusMeasureThread() const
{
  return settings_ctl->focusMeasureThread();
}

void QFocusGraphSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QFocusGraphSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QFocusGraphSettingsDialogBox::closeEvent(QCloseEvent *)
{
  hide();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */
