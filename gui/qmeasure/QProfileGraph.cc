/*
 * QProfileGraph.cc
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */


#include "QProfileGraph.h"
#include <gui/widgets/QToolBarStretch.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>


#define ICON_plot       ":/qmeasure/icons/plot.png"
#define ICON_settings   ":/qmeasure/icons/settings.png"
#define ICON_copy       ":/qmeasure/icons/copy.png"
#define ICON_numberbox  ":/qmeasure/icons/numberbox.png"



template<>
const c_enum_member* members_of<QCPGraph::LineStyle>()
{
  static const c_enum_member members[] = {
    { QCPGraph::LineStyle::lsNone, "None",
        "data points are not connected with any lines (e.g. data only represented "
        "with symbols according to the scatter style" },

    { QCPGraph::LineStyle::lsLine, "Line",
        "data points are connected by a straight line" },

    { QCPGraph::LineStyle::lsStepLeft, "StepLeft",
        "line is drawn as steps where the step height is the value of the left data point" },

    { QCPGraph::LineStyle::lsStepRight, "StepRight",
        "line is drawn as steps where the step height is the value of the right data point" },

    { QCPGraph::LineStyle::lsStepCenter, "StepCenter",
        "line is drawn as steps where the step is in between two data points" },

    { QCPGraph::LineStyle::lsImpulse, "Impulse",
        "each data point is represented by a line parallel to the value axis, "
        "which reaches from the data point to the zero-value-line" },

    { QCPGraph::LineStyle::lsLine }
  };

  return members;
}

namespace {

template<class T>
void get_pixels_(const cv::Mat & image, const cv::Mat & mask,
    const QVector<cv::Point> & pts,
    QVector<double> & keys,
    QVector<double> values[4],
    QVector<uint8_t> ptmasks[4])
{

  keys.clear();

  for( int c = 0; c < 4; ++c ) {
    values[c].clear();
    ptmasks[c].clear();
  }

  const cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  if (mask.empty()) {

    for (int i = 0, n = pts.size(); i < n; ++i) {
      const cv::Point &p = pts[i];
      if (p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows) {

        keys.append(i);

        const T *srcp = src[p.y];
        for (int c = 0; c < cn; ++c) {
          values[c].append(srcp[p.x * cn + c]);
          ptmasks[c].append(true);
        }
      }
    }
  }
  else if (mask.channels() == 1 ) {

    const cv::Mat1b M = mask;

    for( int i = 0, n = pts.size(); i < n; ++i ) {

      const cv::Point & p = pts[i];

      if (p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows) {

        keys.append(i);

        const T * srcp = src[p.y];
        for( int c = 0; c < cn; ++c ) {
          values[c].append(srcp[p.x * cn + c]);
          ptmasks[c].append(M[p.y][p.x]);
        }
      }
    }

  }
  else if (mask.channels() == image.channels() ) {

    for( int i = 0, n = pts.size(); i < n; ++i ) {

      const cv::Point & p = pts[i];

      if (p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows) {

        const T * srcp = src[p.y];
        const uint8_t * mp = mask.ptr<uint8_t>(p.y);

        keys.append(i);

        for( int c = 0; c < cn; ++c ) {
          values[c].append(srcp[p.x * cn + c]);
          ptmasks[c].append(mp[p.x * cn + c]);
        }

      }
    }
  }
  else if (image.channels() == 1 && mask.channels() > 1 ) {

    const cv::Mat1b M = reduce_channels(mask, cv::REDUCE_MAX);

    for( int i = 0, n = pts.size(); i < n; ++i ) {

      const cv::Point & p = pts[i];

      if (p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows) {

        keys.append(i);

        const T * srcp = src[p.y];
        for( int c = 0; c < cn; ++c ) {
          values[c].append(srcp[p.x * cn + c]);
          ptmasks[c].append(M[p.y][p.x]);
        }
      }
    }

  }
}

void get_pixels(const cv::Mat & image, const cv::Mat & mask,
    const QVector<cv::Point> & pts,
    QVector<double> & keys,
    QVector<double> values[4],
    QVector<uint8_t> ptmasks[4])
{
  switch (image.depth()) {
    case CV_8U:
      get_pixels_<uint8_t>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_8S:
      get_pixels_<int8_t>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_16U:
      get_pixels_<uint16_t>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_16S:
      get_pixels_<int16_t>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_32S:
      get_pixels_<int32_t>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_32F:
      get_pixels_<float>(image, mask, pts, keys, values, ptmasks);
      break;
    case CV_64F:
      get_pixels_<double>(image, mask, pts, keys, values, ptmasks);
      break;
  }
}

} // namespace


QProfileGraph::QProfileGraph(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qmeasure_resources);

  setContentsMargins(0, 0, 0, 0);

  _vl = new QVBoxLayout(this);
  _vl->setContentsMargins(0, 0, 0, 0);

  /////////////////////////////////////////////////////////////////////////////////////////////////

  _vl->addWidget(_toolbar = new QToolBar(this));
  _toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  _toolbar->setIconSize(QSize(16, 16));

  _toolbar->addWidget(new QToolBarStretch(_toolbar));

  //

  _toolbar->addAction(_copyToClipboardAction =
      new QAction(getIcon(ICON_copy),
          "Copy To Clipboard..."));

  connect(_copyToClipboardAction, &QAction::triggered,
      this, &ThisClass::onCopyToClipboardActionTriggered);

  _toolbar->addSeparator();

  //

  _toolbar->addAction(_showSettingsAction =
      new QAction(getIcon(ICON_settings),
          "Options..."));

  _showSettingsAction->setCheckable(true);

  connect(_showSettingsAction, &QAction::triggered,
      this, &ThisClass::onShowSettingsActionTriggered);


  //
  _toolbar->addAction(_showStatusbarAction =
      new QAction(getIcon(ICON_numberbox),
          "Statusbar"));

  _showStatusbarAction->setCheckable(true);

  connect(_showStatusbarAction, &QAction::triggered,
      this, &ThisClass::onShowStatusbarActionTriggered);

  /////////////////////////////////////////////////////////////////////////////////////////////////
  _vl->addWidget(_plot = new QCustomPlot(this), 1000);

  static const QColor colors[4] = {
      Qt::blue,
      Qt::green,
      Qt::red,
      Qt::darkYellow,
  };

  for( int i = 0; i < 4; ++i ) {

    QPen pen(colors[i]);
    pen.setWidth(1);
    pen.setCosmetic(true);

    _graphs[i] = _plot->addGraph();
    _graphs[i]->setPen(pen);
    _graphs[i]->setLineStyle(_lineStyle);

    QCPScatterStyle scatterStyle =
        _graphs[i]->scatterStyle();

    scatterStyle.setSize(3);
    scatterStyle.setShape(_lineStyle == QCPGraph::lsNone ?
        QCPScatterStyle::ScatterShape::ssSquare :
        QCPScatterStyle::ScatterShape::ssNone);

    _graphs[i]->setScatterStyle(scatterStyle);

  }


  QPen axis_pen(QColor(150, 150, 150));
  _plot->xAxis->setBasePen(axis_pen);
  _plot->xAxis->setTickPen(axis_pen);
  _plot->xAxis->setSubTickPen(axis_pen);
  _plot->yAxis->setBasePen(axis_pen);
  _plot->yAxis->setTickPen(axis_pen);
  _plot->yAxis->setSubTickPen(axis_pen);
  _plot->xAxis->setRange(0, 120);


  ///////////////////////////////////////////////////////////////////

  // add the text label at the top:

  if( iconStyleSelector().contains("light", Qt::CaseInsensitive) ) {
    _plot->setBackground(QBrush(QColor(0, 0, 0, 0)));
    _plot->xAxis->setTickLabelColor(QColor(255, 255, 255));
    _plot->yAxis->setTickLabelColor(QColor(255, 255, 255));
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
}

void QProfileGraph::showEvent(QShowEvent * event)
{
  Base::showEvent(event);

  const bool visible =
      isVisible();

  if( !visible && plotSettings_ctl && plotSettings_ctl->isVisible() ) {
    plotSettings_ctl->hide();
  }

  Q_EMIT visibilityChanged(visible);
}

void QProfileGraph::hideEvent(QHideEvent * event)
{
  Base::hideEvent(event);

  const bool visible =
      isVisible();

  if( !visible && plotSettings_ctl && plotSettings_ctl->isVisible() ) {
    plotSettings_ctl->hide();
  }

  Q_EMIT visibilityChanged(visible);
}

const QLine & QProfileGraph::currentLine() const
{
  return _currentLine;
}

void QProfileGraph::setLineStyle(QCPGraph::LineStyle v)
{
  if ( _lineStyle != v ) {

    _lineStyle  = v;

    for( int i = 0; i < 4; ++i ) {

      _graphs[i]->setLineStyle(_lineStyle);

      QCPScatterStyle scatterStyle =
          _graphs[i]->scatterStyle();

      scatterStyle.setShape(_lineStyle == QCPGraph::lsNone ?
          QCPScatterStyle::ScatterShape::ssSquare :
          QCPScatterStyle::ScatterShape::ssNone);

      _graphs[i]->setScatterStyle(scatterStyle);

    }

    _plot->replot();
  }
}

QCPGraph::LineStyle QProfileGraph::lineStyle() const
{
  return _lineStyle;
}

void QProfileGraph::setFixXMin(bool v)
{
  _fixXMin = v;
}

bool QProfileGraph::fixXMin() const
{
  return _fixXMin;
}

void QProfileGraph::setFixXMax(bool v)
{
  _fixXMax = v;
}

bool QProfileGraph::fixXMax() const
{
  return _fixXMax;
}

void QProfileGraph::setFixYMin(bool v)
{
  _fixYMin = v;
}

bool QProfileGraph::fixYMin() const
{
  return _fixYMin;
}

void QProfileGraph::setFixYMax(bool v)
{
  _fixYMax = v;
}

bool QProfileGraph::fixYMax() const
{
  return _fixYMax;
}

void QProfileGraph::setSkipZeroPixels(bool v)
{
  if ( _skipZeroPixels != v ) {
    _skipZeroPixels = v;
    replot();
    // Q_EMIT parameterChanged();
  }
}

bool QProfileGraph::skipZeroPixels() const
{
  return _skipZeroPixels;
}

void QProfileGraph::setSkipMaskedPixels(bool v)
{
  if ( _skipMaskedPixels != v ) {
    _skipMaskedPixels = v;
    replot();
    // Q_EMIT parameterChanged();
  }
}

bool QProfileGraph::skipMaskedPixels() const
{
  return _skipMaskedPixels;
}

void QProfileGraph::setXRangeMin(double v)
{
  _plot->xAxis->setRangeLower(v);
  _plot->replot();
}

double QProfileGraph::xRangeMin() const
{
  return _plot->xAxis->range().lower;
}

void QProfileGraph::setXRangeMax(double v)
{
  _plot->xAxis->setRangeUpper(v);
  _plot->replot();
}

double QProfileGraph::xRangeMax() const
{
  return _plot->xAxis->range().upper;
}

void QProfileGraph::setYRangeMin(double v)
{
  _plot->yAxis->setRangeLower(v);
  _plot->replot();
}

double QProfileGraph::yRangeMin() const
{
  return _plot->yAxis->range().lower;
}

void QProfileGraph::setYRangeMax(double v)
{
  _plot->yAxis->setRangeUpper(v);
  _plot->replot();
}

double QProfileGraph::yRangeMax() const
{
  return _plot->yAxis->range().upper;
}


void QProfileGraph::saveParameters(const QString & profileName)
{
  const QString profile =
      profileName.isEmpty() ? QString("QProfileGraph") :
          profileName;

  QSettings settings;

  settings.setValue(QString("%1/lineStyle").arg(profile), (int)lineStyle());
  settings.setValue(QString("%1/fixXMin").arg(profile), fixXMin());
  settings.setValue(QString("%1/fixXMax").arg(profile), fixXMax());
  settings.setValue(QString("%1/fixYMin").arg(profile), fixYMin());
  settings.setValue(QString("%1/fixYMax").arg(profile), fixYMax());
  settings.setValue(QString("%1/skipZeroPixels").arg(profile), skipZeroPixels());
  settings.setValue(QString("%1/skipMaskedPixels").arg(profile), skipMaskedPixels());
  settings.setValue(QString("%1/xRangeMin").arg(profile), xRangeMin());
  settings.setValue(QString("%1/xRangeMax").arg(profile), xRangeMax());
  settings.setValue(QString("%1/yRangeMin").arg(profile), yRangeMin());
  settings.setValue(QString("%1/yRangeMax").arg(profile), yRangeMax());
}

void QProfileGraph::loadParameters(const QString & profileName)
{
  const QString profile =
      profileName.isEmpty() ? QString("QProfileGraph") :
          profileName;

  QSettings settings;

  setLineStyle((QCPGraph::LineStyle)settings.value(QString("%1/lineStyle").arg(profile), lineStyle()).toInt());
  setFixXMin(settings.value(QString("%1/fixXMin").arg(profile), fixXMin()).toBool());
  setFixXMax (settings.value(QString("%1/fixXMax").arg(profile), fixXMax()).toBool());
  setFixYMin (settings.value(QString("%1/fixYMin").arg(profile), fixYMin()).toBool());
  setFixYMax (settings.value(QString("%1/fixYMax").arg(profile), fixYMax()).toBool());
  setSkipZeroPixels (settings.value(QString("%1/skipZeroPixels").arg(profile), skipZeroPixels()).toBool());
  setSkipMaskedPixels (settings.value(QString("%1/skipMaskedPixels").arg(profile), skipMaskedPixels()).toBool());
  setXRangeMin (settings.value(QString("%1/xRangeMin").arg(profile), xRangeMin()).toDouble());
  setXRangeMax (settings.value(QString("%1/xRangeMax").arg(profile), xRangeMax()).toDouble());
  setYRangeMin (settings.value(QString("%1/yRangeMin").arg(profile), yRangeMin()).toDouble());
  setYRangeMax (settings.value(QString("%1/yRangeMax").arg(profile), yRangeMax()).toDouble());
}


void QProfileGraph::showProfilePlot(const QLineF & line, const cv::Mat & image, const cv::Mat & mask)
{
  return showProfilePlot(QLine((int) line.x1(), (int) line.y1(), (int) line.x2(), (int) line.y2()), image, mask);
}

void QProfileGraph::showProfilePlot(const QLine & line, const cv::Mat & image, const cv::Mat & mask)
{
  QVector<cv::Point> pts;

  _current_keys.clear();
  for( int c = 0; c < 4; ++c ) {
    _current_values[c].clear();
    _current_ptmasks[c].clear();
  }

  if( &line != &_currentLine ) {
    _currentLine = line;
  }

  if( !_currentLine.isNull() ) {

    if( line.dx() != 0 || line.dy() != 0 ) {

      const QPoint p1 = line.p1();
      const QPoint p2 = line.p2();

      const int x1 = p1.x();
      const int x2 = p2.x();
      const int y1 = p1.y();
      const int y2 = p2.y();

      if( std::abs(x2 - x1) >= std::abs(y2 - y1) ) {

        const double s =
            (double) (y2 - y1) / (double) (x2 - x1);

        if( x2 >= x1 ) {

          for( int x = x1; x <= x2; ++x ) {
            pts.append(cv::Point(x, qRound(y1 + (x - x1) * s)));
          }
        }
        else {

          for( int x = x1; x >= x2; --x ) {
            pts.append(cv::Point(x, qRound(y1 + (x - x1) * s)));
          }
        }

      }
      else {

        const double s =
            (double) (x2 - x1) / (double) (y2 - y1);

        if( y2 >= y1 ) {

          for( int y = y1; y <= y2; ++y ) {
            pts.append(cv::Point(qRound(x1 + (y - y1) * s), y));
          }
        }
        else {

          for( int y = y1; y >= y2; --y ) {
            pts.append(cv::Point(qRound(x1 + (y - y1) * s), y));
          }
        }
      }
    }

    get_pixels(image, mask, pts,
        _current_keys,
        _current_values,
        _current_ptmasks);
  }

  replot();
}

void QProfileGraph::replot()
{
  for( int c = 0; c < 4; ++c ) {

    if( _current_values[c].empty() ) {
      _graphs[c]->setData(QVector<double>(),
          QVector<double>());
    }

    else if( !_skipZeroPixels && !_skipMaskedPixels ) {

      _graphs[c]->setData(_current_keys,
          _current_values[c]);

    }

    else {

      QVector<double> keys;
      QVector<double> values;

      for( int i = 0; i < _current_values[c].size(); ++i ) {

        const bool skip_this_point =
            (_skipZeroPixels && !_current_values[c][i]) ||
                (_skipMaskedPixels && !_current_ptmasks[c][i]);

        if(  !skip_this_point ) {
          keys.append(_current_keys[i]);
          values.append(_current_values[c][i]);
        }
      }

      _graphs[c]->setData(keys,
          values);
    }

  }

  if( !(_fixXMin && _fixXMax) ) {

    const double xmin =
        _fixXMin ? _plot->xAxis->range().lower :
            0;

    const double xmax =
        _fixXMax ? _plot->xAxis->range().upper :
            _current_keys.size();

    _plot->xAxis->setRange(xmin, xmax);
    Q_EMIT xRangeRescaled();
  }

  if( !(_fixYMin && _fixYMax) ) {

    double ymin =
        _plot->yAxis->range().lower;

    double ymax =
        _plot->yAxis->range().upper;

    _plot->yAxis->rescale();

    const double adjust =
        0.05 * std::abs(_plot->yAxis->range().upper - _plot->yAxis->range().lower);

    if ( !_fixYMin ) {
      ymin = _plot->yAxis->range().lower - adjust;
    }

    if ( !_fixYMax ) {
      ymax = _plot->yAxis->range().upper + adjust;
    }

    _plot->yAxis->setRange(ymin, ymax);

    Q_EMIT yRangeRescaled();
  }

  _plot->replot();
}


void QProfileGraph::onShowSettingsActionTriggered(bool checked)
{
  if ( !checked ) {
    if ( plotSettings_ctl && plotSettings_ctl->isVisible() ) {
      plotSettings_ctl->hide();
    }
  }
  else {

    if( !plotSettings_ctl ) {

      plotSettings_ctl = new QProfileGraphSettingsDialogBox(this);
      plotSettings_ctl->setProfileGraph(this);

      connect(plotSettings_ctl, &QProfileGraphSettingsDialogBox::visibilityChanged,
          [this](bool visible) {
            _showSettingsAction->setChecked(visible);
          });
    }

    plotSettings_ctl->show();
    plotSettings_ctl->setFocus();
  }
}

void QProfileGraph::onCopyToClipboardActionTriggered()
{
  int num_columns = 0;

  for ( int c = 0; c < 4; ++c ) {
    if ( !_current_values[c].isEmpty() ) {
      ++num_columns;
    }
  }

  if ( num_columns < 1 ) {
    CF_DEBUG("No Data available");
    return;
  }

  QClipboard * clipboard =
      QApplication::clipboard();
  if ( !clipboard ) {
    CF_DEBUG("No clipboard available");
    return;
  }


  QString text;

  text.append("X");
  for( int c = 0; c < 4; ++c ) {
    if( !_current_values[c].empty() ) {
      text.append(qsprintf("\tY%d", c));
    }
  }
  text.append("\n");


  for( int i = 0, n = _current_keys.size(); i < n; ++i ) {

    if ( !_skipZeroPixels && !_skipMaskedPixels ) {

      text.append(qsprintf("%g", _current_keys[i]));

      for( int c = 0; c < 4; ++c ) {
        if( i < _current_values[c].size() ) {
          text.append(qsprintf("\t%g", _current_values[c][i]));
        }
      }

      text.append("\n");
    }
    else {

      bool key_added = false;

      for( int c = 0; c < 4; ++c ) {

        if( i < _current_values[c].size() ) {

          const bool skip_this_point =
              (_skipZeroPixels && !_current_values[c][i]) ||
                  (_skipMaskedPixels && !_current_ptmasks[c][i]);

          if( !skip_this_point ) {

            if( !key_added ) {
              text.append(qsprintf("%g", _current_keys[i]));
              key_added = true;
            }

            text.append(qsprintf("\t%g", _current_values[c][i]));
          }
        }
      }

      if( key_added ) {
        text.append("\n");
      }

    }
  }


  clipboard->setText(text);
}


void QProfileGraph::onShowStatusbarActionTriggered(bool checked)
{
  if( checked ) {

    if( !_statusBar ) {
      _vl->addWidget(_statusBar = new QStatusBar(this), 0, Qt::AlignBottom);
      _statusBar->setSizeGripEnabled(true);
    }

    _statusBar->show();
    _statusBar->showMessage("");

    connect(_plot, &QCustomPlot::mouseMove,
        this, &ThisClass::onCustomPlotMouseMove);

  }
  else if( _statusBar ) {

    _statusBar->hide();

    disconnect(_plot, &QCustomPlot::mouseMove,
        this, &ThisClass::onCustomPlotMouseMove);

  }

}


void QProfileGraph::onCustomPlotMouseMove(QMouseEvent * e)
{
  if ( _statusBar && _statusBar->isVisible() ) {

    const double x =
        _plot->xAxis->pixelToCoord(e->pos().x());

    const double y =
        _plot->yAxis->pixelToCoord(e->pos().y());

    _statusBar->showMessage(qsprintf("x=%g y=%g", x, y));
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphSettings::QProfileGraphSettings(QWidget * parent) :
    Base("QProfileGraphSettings", parent)
{
  lineStyle_ctl =
      add_enum_combobox<QCPGraph::LineStyle>("Line Style:",
          "Set line style",
          [this](QCPGraph::LineStyle v) {
            if ( options_ ) {
              options_->setLineStyle(v);
            }
          },
          [this](QCPGraph::LineStyle * v) {
            if ( options_ ) {
              *v = options_->lineStyle();
              return true;
            }
            return false;
          });

  fixXMin_ctl =
      add_checkbox("Fix X min:",
          "Set checked to fix X min range of the plot",
          [this](bool checked) {
            if ( options_ ) {
              options_->setFixXMin(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->fixXMin();
              return true;
            }
            return false;
          });

  xRangeMin_ctl =
      add_numeric_box<double>("Xmin:",
          "X range minimum value",
          [this](double v) {
            if ( options_ ) {
              options_->setXRangeMin(v);
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->xRangeMin();
              return true;
            }
            return false;
          });


  fixXMax_ctl =
      add_checkbox("Fix X max:",
          "Set checked to fix X maxx range of the plot",
          [this](bool checked) {
            if ( options_ ) {
              options_->setFixXMax(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->fixXMax();
              return true;
            }
            return false;
          });

  xRangeMax_ctl =
      add_numeric_box<double>("Xmax:",
          "X range maximum value",
          [this](double v) {
            if ( options_ ) {
              options_->setXRangeMax(v);
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->xRangeMax();
              return true;
            }
            return false;
          });

  ///

  fixYMin_ctl =
      add_checkbox("Fix Y min:",
          "Set checked to fix Y min range of the plot",
          [this](bool checked) {
            if ( options_ ) {
              options_->setFixYMin(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->fixYMin();
              return true;
            }
            return false;
          });


  yRangeMin_ctl =
      add_numeric_box<double>("Ymin:",
          "Y range minimum value",
          [this](double v) {
            if ( options_ ) {
              options_->setYRangeMin(v);
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->yRangeMin();
              return true;
            }
            return false;
          });


  fixYMax_ctl =
      add_checkbox("Fix Y Max:",
          "Set checked to fix Y max range of the plot",
          [this](bool checked) {
            if ( options_ ) {
              options_->setFixYMax(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->fixYMax();
              return true;
            }
            return false;
          });

  yRangeMax_ctl =
      add_numeric_box<double>("Ymax:",
          "Y range maximum value",
          [this](double v) {
            if ( options_ ) {
              options_->setYRangeMax(v);
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->yRangeMax();
              return true;
            }
            return false;
          });

  ///

  skipZeroPixels_ctl =
      add_checkbox("Skip Zero values:",
          "Set checked to skip pixels with zero value",
          [this](bool checked) {
            if ( options_ ) {
              options_->setSkipZeroPixels(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->skipZeroPixels();
              return true;
            }
            return false;
          });

  skipMaskedPixels_ctl =
      add_checkbox("Skip Masked values:",
          "Set checked to skip masked pixels",
          [this](bool checked) {
            if ( options_ ) {
              options_->setSkipMaskedPixels(checked);
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->skipMaskedPixels();
              return true;
            }
            return false;
          });


  ///

  updateControls();
}

void QProfileGraphSettings::set_options(QProfileGraph * profileGraph)
{
  setProfileGraph(profileGraph);
}

void QProfileGraphSettings::setProfileGraph(QProfileGraph * options)
{
  if ( options_ ) {
    options_->disconnect(this);
  }

  if ( (options_ = options) ) {

    connect(options_, &QProfileGraph::xRangeRescaled,
        [this]() {
          c_update_controls_lock lock(this);
          xRangeMin_ctl->setValue(options_->xRangeMin());
          xRangeMax_ctl->setValue(options_->xRangeMax());
        });

    connect(options_, &QProfileGraph::yRangeRescaled,
        [this]() {
          c_update_controls_lock lock(this);
          yRangeMin_ctl->setValue(options_->yRangeMin());
          yRangeMax_ctl->setValue(options_->yRangeMax());
        });

    //    connect(profileGraph_, &QProfileGraph::parameterChanged,
    //        this, &ThisClass::parameterChanged);

  }

  updateControls();
}

QProfileGraph * QProfileGraphSettings::profileGraph() const
{
  return options_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphDialogBox::QProfileGraphDialogBox(QWidget * parent) :
    ThisClass("Pixel Intensity Profile", parent)
{
}

QProfileGraphDialogBox::QProfileGraphDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);
  resize(320, 240);

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

///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphSettingsDialogBox::QProfileGraphSettingsDialogBox(QWidget * parent) :
    ThisClass("Profile Graph Settings", parent)
{

}

QProfileGraphSettingsDialogBox::QProfileGraphSettingsDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);
  // resize(320, 240);

  QVBoxLayout * lv =
      new QVBoxLayout(this);

  lv->setContentsMargins(0,0,0,0);

  lv->addWidget(_settingsWidget =
      new QProfileGraphSettings(parent));
}

void QProfileGraphSettingsDialogBox::setProfileGraph(QProfileGraph * profileGraph)
{
  return _settingsWidget->setProfileGraph(profileGraph);
}

QProfileGraph * QProfileGraphSettingsDialogBox::profileGraph() const
{
  return _settingsWidget->profileGraph();
}

QProfileGraphSettings * QProfileGraphSettingsDialogBox::settingsWidget() const
{
  return _settingsWidget;
}

void QProfileGraphSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QProfileGraphSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

