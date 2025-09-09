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
#define ICON_clear      ":/qmeasure/icons/clear_measurements.png"


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
  textLabel_ = new QCPItemText(plot_);
  textLabel_->position->setType(QCPItemPosition::ptAxisRectRatio);
  textLabel_->setPositionAlignment(Qt::AlignLeft | Qt::AlignTop);
  textLabel_->position->setCoords(0.05, 0.05); // lower right corner of axis rect
  textLabel_->setText("");
  textLabel_->setTextAlignment(Qt::AlignLeft);
  textLabel_->setFont(QFont(font().family(), 12));
  textLabel_->setPadding(QMargins(8, 0, 0, 0));

  ///////////////////////////////////////////////////////////////////

  if( iconStyleSelector().contains("light", Qt::CaseInsensitive) ) {
    plot_->setBackground(QBrush(QColor(0, 0, 0, 0)));
    plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
    plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));
    textLabel_->setColor(Qt::yellow);
  }

}

void QMeasureGraph::set_max_measurements(int v)
{
  _max_measurements = v;

  while ( _measured_values.size() > _max_measurements ) {
    _measured_values.pop_front();
  }
}

int QMeasureGraph::max_measurements() const
{
  return _max_measurements;
}

void QMeasureGraph::setCurrentMeasure(QMeasure * cm)
{
  _cm.clear();
  if ( cm ) {
    _cm.emplace(cm);
  }

  updateEnableMeasurements();

  if ( _cm.empty() ) {
    clearGraphs();
  }
  else {
    updateGraphs();
  }
}

void QMeasureGraph::clearMeasurements()
{
  if ( !_measured_values.empty() ) {
    _measured_values.clear();
    updateGraphs();
  }
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

void QMeasureGraph::onFramesMeasured(const QList<QMeasureProvider::MeasuredFrame> & frames)
{
  if ( _cm.size() != 1 ) {
    CF_ERROR("APP BUG: _cm.size()=%zu", _cm.size());
    return;
  }

  const QMeasure * const mp = *_cm.begin();
  bool hasChanges = false;

  for ( const auto & frame : frames ) {
    if ( frame.dataChannel.isEmpty() ) {

      for ( const auto & m : frame.measurements ) {
        if ( m.measure == mp && m.cn > 0 ) {

          _measured_values.emplace_back(m);
          while ( _measured_values.size() > _max_measurements ) {
            _measured_values.pop_front();
          }
          hasChanges = true;
          break;
        }
      }
      break;
    }
  }

  if ( hasChanges ) {
    updateGraphs();
  }

}

void QMeasureGraph::updateGraphs()
{
  QVector<double> keys[4];
  QVector<double> values[4];
  const QMeasureProvider:: MeasuredValue * mlast = nullptr;

  for (int i = 0, n = _measured_values.size(); i < n; ++i) {
    const auto &mv = _measured_values[i];
    for (int c = 0; c < mv.cn; ++c) {
      keys[c].append(i);
      values[c].append(mv.value(c));
      mlast = &mv;
    }
  }

  for (int i = 0; i < 4; ++i) {
    graphs_[i]->setData(keys[i], values[i], true);
  }

  if (!mlast) {
    textLabel_->setText("");
  }
  else {
    switch (mlast->cn) {
    case 0:
      textLabel_->setText("");
    break;
    case 1:
      textLabel_->setText(ssprintf("%+g",
          mlast->value[0]).c_str());
    break;
    case 2:
      textLabel_->setText(ssprintf("%+g %+g",
          mlast->value[0],
          mlast->value[1]).c_str());
    break;
    case 3:
      textLabel_->setText(ssprintf("%+g %+g %+g",
          mlast->value[0],
          mlast->value[1],
          mlast->value[2]).c_str());
    break;
    case 4:
      textLabel_->setText(ssprintf("%+g %+g %+g %+g",
          mlast->value[0],
          mlast->value[1],
          mlast->value[2],
          mlast->value[3]).c_str());
    break;
    }
  }

  plot_->yAxis->rescale();
  plot_->xAxis->setRange(0, std::max(10., 1.2 * _measured_values.size()));
  plot_->replot();

//  using Frame = QMeasureProvider::MeasuredFrame;
//  using MeasuredValue = QMeasureProvider::MeasuredValue;
//
//  if( !_cm.empty() ) {
//
//    QVector<double> keys[4];
//    QVector<double> values[4];
//    int max_key = 0;
//    int i = 0;
//
//    const QMeasure * cm = *_cm.begin();
//    const MeasuredValue * mlast = nullptr;
//
//
//    for( auto ii = QMeasureProvider::measured_frames().begin();
//        ii != QMeasureProvider::measured_frames().end();
//        ++i, ++ii ) {
//
//        for( const auto &m : ii->measurements ) {
//          if ( m.measure == cm && m.cn > 0 ) {
//            for ( int j = 0; j < m.cn; ++j ) {
//              mlast = &m;
//              keys[j].append(i);
//              values[j].append(m.value(j));
//              if ( i > max_key ) {
//                max_key = i;
//              }
//            }
//          }
//        }
//      }
//
//      for( i = 0; i < 4; ++i ) {
//        graphs_[i]->setData(keys[i], values[i], true);
//      }
//
//      if ( !mlast ) {
//        textLabel_->setText("");
//      }
//      else {
//        switch(mlast->cn) {
//          case 0:
//            textLabel_->setText("");
//            break;
//          case 1:
//            textLabel_->setText(ssprintf("%+g",
//                mlast->value[0]).c_str());
//            break;
//          case 2:
//            textLabel_->setText(ssprintf("%+g %+g",
//                mlast->value[0],
//                mlast->value[1]).c_str());
//            break;
//          case 3:
//            textLabel_->setText(ssprintf("%+g %+g %+g",
//                mlast->value[0],
//                mlast->value[1],
//                mlast->value[2]).c_str());
//            break;
//          case 4:
//            textLabel_->setText(ssprintf("%+g %+g %+g %+g",
//                mlast->value[0],
//                mlast->value[1],
//                mlast->value[2],
//                mlast->value[3]).c_str());
//            break;
//        }
//      }
//
//
//      plot_->yAxis->rescale();
//      plot_->xAxis->setRange(0, 1.2 * max_key);
//
//      //      enum COLORID colorid = provider_->colorid();
//      //      if( colorid != last_colorid_ ) {
//      //        updatePenColors(last_colorid_ = colorid);
//      //      }
//
//      plot_->replot();
//  }
}

void QMeasureGraph::showEvent(QShowEvent * event)
{
  Base::showEvent(event);
  updateEnableMeasurements();
}

void QMeasureGraph::hideEvent(QHideEvent * event)
{
  Base::hideEvent(event);
  updateEnableMeasurements();
}

void QMeasureGraph::updateEnableMeasurements()
{
  const bool enable =
      !_cm.empty() &&
          this->isVisible();

  if (enable != _measurementsEnabled) {
    if (!(_measurementsEnabled = enable)) {
      QMeasureProvider::instance()->disconnect(this);
    }
    else {
      connect(QMeasureProvider::instance(), &QMeasureProvider::framesMeasured,
          this, &ThisClass::onFramesMeasured);
    }
  }

  if ( _measurementsEnabled ) {
    QMeasureProvider::request_measures(&_cm);
  }
  else {
    QMeasureProvider::remove_measure_request(&_cm);
  }

//  if( enable ) {
//    QMeasureProvider::request_measures(&_cm);
//    connect(QMeasureProvider::instance(), &QMeasureProvider::measurementsChanged,
//        this, &ThisClass::updateGraphs);
//  }
//  else {
//    QMeasureProvider::instance()->disconnect(this);
//    QMeasureProvider::remove_measure_request(&_cm);
//  }
}


namespace {

class QMeasureGraphCombo:
    public QMeasureSelectionCombo
{
public:
  typedef QMeasureGraphCombo ThisClass;
  typedef QMeasureSelectionCombo Base;

  QMeasureGraphCombo(QWidget * parent = nullptr) : Base(parent)
  {
    insertItem(0, "NONE", QVariant::fromValue((QMeasure*) nullptr));
    setCurrentIndex(0);
  }

};

} // namespace

QMeasureGraphDock::QMeasureGraphDock(const QString & title, QWidget * parent, QMeasureGraph * graph) :
    Base(title, parent, graph)
{
  if( graph ) {

    QCustomDockTitleBar *bar =
        titleBar();

    bar->addButton(buttonClear_ctl = new QToolButton(this));
    buttonClear_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
    buttonClear_ctl->setIcon(getIcon(ICON_clear));
    buttonClear_ctl->setText("clear");
    buttonClear_ctl->setToolTip("Clear measurements");
    connect(buttonClear_ctl, &QToolButton::clicked,
        graph, &QMeasureGraph::clearMeasurements);


    bar->addWidget(maxMeasurements_ctl = new QNumericBox(this));
    maxMeasurements_ctl->setValue(graph->max_measurements());
    connect(maxMeasurements_ctl, &QNumericBox::textChanged,
        [this, graph]() {
          int v;
          if ( fromString(maxMeasurements_ctl->text(), &v) ) {
            graph->set_max_measurements(v);
          }
      });

    bar->addWidget(combobox_ctl = new QMeasureGraphCombo());
    connect(combobox_ctl, &QMeasureSelectionCombo::currentMeasureChanged,
        [this, graph]() {
          graph->setCurrentMeasure(combobox_ctl->currentMeasure());
        });
  }
}
