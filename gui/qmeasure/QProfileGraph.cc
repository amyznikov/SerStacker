/*
 * QProfileGraph.cc
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#include "QProfileGraph.h"
#include <gui/widgets/QToolBarStretch.h>
#include <gui/widgets/style.h>
#include <core/ssprintf.h>
#include <core/debug.h>


#define ICON_plot       ":/qmeasure/icons/plot.png"
#define ICON_settings   ":/qmeasure/icons/settings.png"
#define ICON_copy       ":/qmeasure/icons/copy.png"


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
    QVector<uint8_t> & ptmasks)
{
  const cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  const cv::Mat1b M =
      mask.empty() ? cv::Mat1b() :
          mask;

  keys.clear();

  for( int c = 0; c < 4; ++c ) {
    values[c].clear();
  }

  for( int i = 0, n = pts.size(); i < n; ++i ) {

    const cv::Point & p =
        pts[i];

    if( p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows ) {

      const T * srcp = src[p.y];

      keys.append(i);

      if( !M.empty() ) {
        ptmasks.append(M[p.y][p.x]);
      }
      else {
        ptmasks.append(true);
      }

      for( int c = 0; c < cn; ++c ) {

        const T & value =
            srcp[p.x * cn + c];

        values[c].append(value);
      }
    }
  }
}

void get_pixels(const cv::Mat & image, const cv::Mat & mask,
    const QVector<cv::Point> & pts,
    QVector<double> & keys,
    QVector<double> values[4],
    QVector<uint8_t> & ptmasks)
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

  vl_ = new QVBoxLayout(this);
  vl_->setContentsMargins(0,0,0,0);

  /////////////////////////////////////////////////////////////////////////////////////////////////

  vl_->addWidget(toolbar_ = new QToolBar(this));
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setIconSize(QSize(16, 16));

  toolbar_->addWidget(new QToolBarStretch(toolbar_));


  toolbar_->addAction(copyToClipboardAction_ =
      new QAction(getIcon(ICON_copy),
          "Copy To Clipboard..."));

  connect(copyToClipboardAction_, &QAction::triggered,
      this, &ThisClass::onCopyToClipboardActionTriggered);

  toolbar_->addSeparator();

  //addStretch(toolbar_);

  toolbar_->addAction(showSettingsAction_ =
      new QAction(getIcon(ICON_settings),
          "Options..."));

  showSettingsAction_->setCheckable(true);

  connect(showSettingsAction_, &QAction::triggered,
      this, &ThisClass::onShowSettingsActionTriggered);




  /////////////////////////////////////////////////////////////////////////////////////////////////

  vl_->addWidget(plot_ = new QCustomPlot(this));

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

    graphs_[i] = plot_->addGraph();
    graphs_[i]->setPen(pen);
    graphs_[i]->setLineStyle(lineStyle_);

    QCPScatterStyle scatterStyle =
        graphs_[i]->scatterStyle();

    scatterStyle.setSize(3);
    scatterStyle.setShape(lineStyle_ == QCPGraph::lsNone ?
        QCPScatterStyle::ScatterShape::ssSquare :
        QCPScatterStyle::ScatterShape::ssNone);

    graphs_[i]->setScatterStyle(scatterStyle);

  }


  QPen axis_pen(QColor(150, 150, 150));
  plot_->xAxis->setBasePen(axis_pen);
  plot_->xAxis->setTickPen(axis_pen);
  plot_->xAxis->setSubTickPen(axis_pen);
  plot_->yAxis->setBasePen(axis_pen);
  plot_->yAxis->setTickPen(axis_pen);
  plot_->yAxis->setSubTickPen(axis_pen);
  plot_->xAxis->setRange(0, 120);


  ///////////////////////////////////////////////////////////////////

  // add the text label at the top:

  if( iconStyleSelector().contains("light", Qt::CaseInsensitive) ) {
    plot_->setBackground(QBrush(QColor(0, 0, 0, 0)));
    plot_->xAxis->setTickLabelColor(QColor(255, 255, 255));
    plot_->yAxis->setTickLabelColor(QColor(255, 255, 255));
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
  return currentLine_;
}

void QProfileGraph::setLineStyle(QCPGraph::LineStyle v)
{
  if ( lineStyle_ != v ) {

    lineStyle_  = v;

    for( int i = 0; i < 4; ++i ) {

      graphs_[i]->setLineStyle(lineStyle_);

      QCPScatterStyle scatterStyle =
          graphs_[i]->scatterStyle();

      scatterStyle.setShape(lineStyle_ == QCPGraph::lsNone ?
          QCPScatterStyle::ScatterShape::ssSquare :
          QCPScatterStyle::ScatterShape::ssNone);

      graphs_[i]->setScatterStyle(scatterStyle);

    }

    plot_->replot();
  }
}

QCPGraph::LineStyle QProfileGraph::lineStyle() const
{
  return lineStyle_;
}

void QProfileGraph::setFixXMin(bool v)
{
  fixXMin_ = v;
}

bool QProfileGraph::fixXMin() const
{
  return fixXMin_;
}

void QProfileGraph::setFixXMax(bool v)
{
  fixXMax_ = v;
}

bool QProfileGraph::fixXMax() const
{
  return fixXMax_;
}

void QProfileGraph::setFixYMin(bool v)
{
  fixYMin_ = v;
}

bool QProfileGraph::fixYMin() const
{
  return fixYMin_;
}

void QProfileGraph::setFixYMax(bool v)
{
  fixYMax_ = v;
}

bool QProfileGraph::fixYMax() const
{
  return fixYMax_;
}

void QProfileGraph::setSkipZeroPixels(bool v)
{
  if ( skipZeroPixels_ != v ) {
    skipZeroPixels_ = v;
    replot();
    // Q_EMIT parameterChanged();
  }
}

bool QProfileGraph::skipZeroPixels() const
{
  return skipZeroPixels_;
}

void QProfileGraph::setSkipMaskedPixels(bool v)
{
  if ( skipMaskedPixels_ != v ) {
    skipMaskedPixels_ = v;
    replot();
    // Q_EMIT parameterChanged();
  }
}

bool QProfileGraph::skipMaskedPixels() const
{
  return skipMaskedPixels_;
}

void QProfileGraph::setXRangeMin(double v)
{
  plot_->xAxis->setRangeLower(v);
  plot_->replot();
}

double QProfileGraph::xRangeMin() const
{
  return plot_->xAxis->range().lower;
}

void QProfileGraph::setXRangeMax(double v)
{
  plot_->xAxis->setRangeUpper(v);
  plot_->replot();
}

double QProfileGraph::xRangeMax() const
{
  return plot_->xAxis->range().upper;
}

void QProfileGraph::setYRangeMin(double v)
{
  plot_->yAxis->setRangeLower(v);
  plot_->replot();
}

double QProfileGraph::yRangeMin() const
{
  return plot_->yAxis->range().lower;
}

void QProfileGraph::setYRangeMax(double v)
{
  plot_->yAxis->setRangeUpper(v);
  plot_->replot();
}

double QProfileGraph::yRangeMax() const
{
  return plot_->yAxis->range().upper;
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

  current_keys_.clear();
  current_ptmasks_.clear();
  for( int c = 0; c < 4; ++c ) {
    current_values_[c].clear();
  }

  if( &line != &currentLine_ ) {
    currentLine_ = line;
  }

  if( !currentLine_.isNull() ) {

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
        current_keys_,
        current_values_,
        current_ptmasks_);
  }

  replot();
}

void QProfileGraph::replot()
{
  for( int c = 0; c < 4; ++c ) {

    if( current_values_[c].empty() ) {
      graphs_[c]->setData(QVector<double>(),
          QVector<double>());
    }

    else if( !skipZeroPixels_ && !skipMaskedPixels_ ) {

      graphs_[c]->setData(current_keys_,
          current_values_[c]);

    }

    else {

      QVector<double> keys;
      QVector<double> values;

      for( int i = 0; i < current_values_[c].size(); ++i ) {

        const bool skip_this_point =
            (skipZeroPixels_ && !current_values_[c][i]) ||
                (skipMaskedPixels_ && !current_ptmasks_[i]);

        if(  !skip_this_point ) {
          keys.append(current_keys_[i]);
          values.append(current_values_[c][i]);
        }
      }

      graphs_[c]->setData(keys,
          values);
    }

  }

  if( !(fixXMin_ && fixXMax_) ) {

    const double xmin =
        fixXMin_ ? plot_->xAxis->range().lower :
            0;

    const double xmax =
        fixXMax_ ? plot_->xAxis->range().upper :
            current_keys_.size();

    plot_->xAxis->setRange(xmin, xmax);
    Q_EMIT xRangeRescaled();
  }

  if( !(fixYMin_ && fixXMax_) ) {

    double ymin =
        plot_->yAxis->range().lower;

    double ymax =
        plot_->yAxis->range().upper;

    plot_->yAxis->rescale();

    if ( !fixYMin_ ) {
      ymin = plot_->yAxis->range().lower;
    }

    if ( !fixYMax_ ) {
      ymax = plot_->yAxis->range().upper;
    }

    plot_->yAxis->setRange(ymin, ymax);

    Q_EMIT yRangeRescaled();
  }

  plot_->replot();
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
            showSettingsAction_->setChecked(visible);
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
    if ( !current_values_[c].isEmpty() ) {
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
    if( !current_values_[c].empty() ) {
      text.append(qsprintf("\tY%d", c));
    }
  }
  text.append("\n");


  for( int i = 0, n = current_keys_.size(); i < n; ++i ) {

    if ( !skipZeroPixels_ && !skipMaskedPixels_ ) {

      text.append(qsprintf("%g", current_keys_[i]));

      for( int c = 0; c < 4; ++c ) {
        if( i < current_values_[c].size() ) {
          text.append(qsprintf("\t%g", current_values_[c][i]));
        }
      }

      text.append("\n");
    }
    else {

      bool key_added = false;

      for( int c = 0; c < 4; ++c ) {

        if( i < current_values_[c].size() ) {

          const bool skip_this_point =
              (skipZeroPixels_ && !current_values_[c][i]) ||
                  (skipMaskedPixels_ && !current_ptmasks_[i]);

          if( !skip_this_point ) {

            if( !key_added ) {
              text.append(qsprintf("%g", current_keys_[i]));
              key_added = true;
            }

            text.append(qsprintf("\t%g", current_values_[c][i]));
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
//
//void QProfileGraphSettings::onupdatecontrols()
//{
//  if ( !options_ ) {
//    setEnabled(false);
//  }
//  else {
//    Base::onupdatecontrols();
//    setEnabled(true);
//  }
//}

//void QProfileGraphSettings::onload(QSettings & settings)
//{
//  // Base::onload(settings);
//}


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

  lv->addWidget(settingsWidget_ =
      new QProfileGraphSettings(parent));
}

void QProfileGraphSettingsDialogBox::setProfileGraph(QProfileGraph * profileGraph)
{
  return settingsWidget_->setProfileGraph(profileGraph);
}

QProfileGraph * QProfileGraphSettingsDialogBox::profileGraph() const
{
  return settingsWidget_->profileGraph();
}

QProfileGraphSettings * QProfileGraphSettingsDialogBox::settingsWidget() const
{
  return settingsWidget_;
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


