/*
 * QFocusGraph.cc
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#include "QFocusGraph.h"
#include <gui/widgets/style.h>
#include <core/ssprintf.h>


#define ICON_menu         ":/qfocus/icons/menu.png"
#define ICON_chart        ":/qfocus/icons/chart.png"
#define ICON_roi          ":/qfocus/icons/roi.png"
#define ICON_options      ":/qfocus/icons/options.png"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFocusGraph::QFocusGraph(QWidget * parent) :
    Base(parent)
{
  Q_INIT_RESOURCE(qfocus_resources);

  vl_ = new QVBoxLayout(this);
  vl_->addWidget(plot_ = new QCustomPlot(this));

  QPen pen(Qt::red);
  pen.setWidth(2);
  pen.setCosmetic(true);

  for( int i = 0; i < QFocusMeasureProvider::MAX_CHANNELS; ++i ) {
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
          settings_ctl->setFocusMeasureProvider(provider_);

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
        if ( provider_ ) {
          provider_->setEnabled(checked && isVisible());
        }
      });


  ///////////////////////////////////////////////////////////////////
}

void QFocusGraph::setFocusMeasureProvider(QFocusMeasureProvider * provider)
{
  if( provider_ ) {
    provider_->disconnect(this);
  }

  if( (provider_ = provider) ) {

    connect(provider_, &QFocusMeasureProvider::dataChanged,
        this, &ThisClass::updateFocusGraph,
        Qt::QueuedConnection);

    provider_->setEnabled(isVisible() && enableFocusTrackAction_->isChecked());
  }

  if( settings_ctl ) {
    settings_ctl->setFocusMeasureProvider(provider_);
  }
}

QFocusMeasureProvider * QFocusGraph::focusMeasureProvider() const
{
  return provider_;
}


void QFocusGraph::clearFocusGraph()
{
  static const QVector<double> empty_keys;
  static const QVector<double> empty_values;

  for( int i = 0; i < QFocusMeasureProvider::MAX_CHANNELS; ++i ) {
    graphs_[i]->setData(empty_keys, empty_values);
  }

  plot_->replot();
}

void QFocusGraph::updateFocusGraph()
{
  if ( provider_ ) {

    QMutexLocker lock(&provider_->mutex());

    QVector<double> keys;

    for( int i = 0; i < QFocusMeasureProvider::MAX_CHANNELS; ++i ) {

      const QVector<double> &values =
          provider_->measurements(i);

      keys.clear();

      for( int j = 0, m = values.size(); j < m; ++j ) {
        keys.append(j);
      }

      graphs_[i]->setData(keys, values, true);
    }

    plot_->yAxis->rescale();


    enum COLORID colorid = provider_->colorid();
    if ( colorid != last_colorid_ )  {
      updatePenColors(last_colorid_ = colorid);
    }

    plot_->replot();
  }
}

void QFocusGraph::updatePenColors(enum COLORID colorid)
{
  static const QColor *selectedColors = nullptr;

  if( is_bayer_pattern(colorid) ) {
    // extract_bayer_planes() always orders output channels as[ R G1 B G2 ].
    static const QColor colors[QFocusMeasureProvider::MAX_CHANNELS] = {
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
        static const QColor colors[QFocusMeasureProvider::MAX_CHANNELS] = {
            Qt::lightGray,
            Qt::lightGray,
            Qt::lightGray,
            Qt::lightGray,
        };
        selectedColors = colors;
      }
        break;

      case COLORID_RGB: {
        static const QColor colors[QFocusMeasureProvider::MAX_CHANNELS] = {
            Qt::red,
            Qt::green,
            Qt::blue,
            Qt::yellow,
        };
        selectedColors = colors;
      }
        break;

      case COLORID_BGR: {
        static const QColor colors[QFocusMeasureProvider::MAX_CHANNELS] = {
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
        static const QColor colors[QFocusMeasureProvider::MAX_CHANNELS] = {
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

  for( int i = 0; i < QFocusMeasureProvider::MAX_CHANNELS; ++i ) {
    pen.setColor(selectedColors[i]);
    graphs_[i]->setPen(pen);
  }
}


void QFocusGraph::showEvent(QShowEvent *event)
{
  Base::showEvent(event);

  if ( provider_ ) {
    provider_->setEnabled(enableFocusTrackAction_->isChecked() && isVisible());
  }

}

void QFocusGraph::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  if ( provider_ ) {
    provider_->setEnabled(enableFocusTrackAction_->isChecked() && isVisible());
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
            if ( provider_ ) {
              provider_->measure().set_eps(v);
              provider_->save_parameters();
            }
          },
          [this](double * v) {
            if ( provider_ ) {
              *v = provider_->measure().eps();
              return true;
            }
            return false;
          });

  dscale_ctl =
      add_numeric_box<int>(
          "dscale",
          [this](int v) {
            if ( provider_ ) {
              provider_->measure().set_dscale(v);
              provider_->save_parameters();
            }
          },
          [this](int * v) {
            if ( provider_ ) {
              *v = provider_->measure().dscale();
              return true;
            }
            return false;
          });

  avgchannel_ctl =
      add_checkbox(
          "avgchannel",
          [this](bool v ) {
            if ( provider_ ) {
              provider_->measure().set_avgchannel(v);
              provider_->save_parameters();
            }
          },
          [this](bool * v) {
            if ( provider_ ) {
              *v = provider_->measure().avgchannel();
              return true;
            }
            return false;
          });


  updateControls();
}

void QFocusGraphSettingsWidget::setFocusMeasureProvider(QFocusMeasureProvider * provider)
{
  provider_ = provider;
  updateControls();
}

QFocusMeasureProvider * QFocusGraphSettingsWidget::focusMeasureProvider() const
{
  return provider_;
}

void QFocusGraphSettingsWidget::onupdatecontrols()
{
  Base::onupdatecontrols();

  if ( !provider_ ) {
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

void QFocusGraphSettingsDialogBox::setFocusMeasureProvider(QFocusMeasureProvider * provider)
{
  settings_ctl->setFocusMeasureProvider(provider);
}

QFocusMeasureProvider * QFocusGraphSettingsDialogBox::focusMeasureProvider() const
{
  return settings_ctl->focusMeasureProvider();
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
