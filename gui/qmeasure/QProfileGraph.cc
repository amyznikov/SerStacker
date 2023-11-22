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
  static constexpr c_enum_member members[] = {
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
void get_pixels_(const cv::Mat & image, const QVector<cv::Point> & pts, bool skip_zeros,
    QVector<double> keys[4],
    QVector<double> values[4])
{
  const cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  for( int c = 0; c < 4; ++c ) {
    values[c].clear();
    keys[c].clear();
  }

  if( skip_zeros ) {

    for( int i = 0, n = pts.size(); i < n; ++i ) {

      const cv::Point &p =
          pts[i];

      if( p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows ) {

        const T *srcp = src[p.y];

        for( int c = 0; c < cn; ++c ) {

          const T &value =
              srcp[p.x * cn + c];

          if( value != 0 ) {
            keys[c].append(i);
            values[c].append(value);
          }
        }

      }
    }

  }
  else {

    for( int i = 0, n = pts.size(); i < n; ++i ) {

      const cv::Point &p =
          pts[i];

      if( p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows ) {

        const T *srcp = src[p.y];

        keys[0].append(i);

        for( int c = 0; c < cn; ++c ) {

          const T &value =
              srcp[p.x * cn + c];

          values[c].append(value);
        }
      }
    }
  }
}

void get_pixels(const cv::Mat & image, const QVector<cv::Point> & pts, bool skip_zeros,
    QVector<double> keys[4], QVector<double> values[4])
{
  switch (image.depth()) {
    case CV_8U:
      get_pixels_<uint8_t>(image, pts, skip_zeros, keys, values);
      break;
    case CV_8S:
      get_pixels_<int8_t>(image, pts, skip_zeros, keys, values);
      break;
    case CV_16U:
      get_pixels_<uint16_t>(image, pts, skip_zeros, keys, values);
      break;
    case CV_16S:
      get_pixels_<int16_t>(image, pts, skip_zeros, keys, values);
      break;
    case CV_32S:
      get_pixels_<int32_t>(image, pts, skip_zeros, keys, values);
      break;
    case CV_32F:
      get_pixels_<float>(image, pts, skip_zeros, keys, values);
      break;
    case CV_64F:
      get_pixels_<double>(image, pts, skip_zeros, keys, values);
      break;
  }
}


//QWidget* addStretch(QToolBar * toolbar)
//{
//  QWidget *stretch = new QWidget(toolbar);
//  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
//  stretch->setBackgroundRole(QPalette::NoRole);
//  toolbar->addWidget(stretch);
//  return stretch;
//}

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

void QProfileGraph::replot()
{
  plot_->replot();
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

void QProfileGraph::setFixXRange(bool v)
{
  fixXRange_ = v;
}

bool QProfileGraph::fixXRange() const
{
  return fixXRange_;
}

void QProfileGraph::setFixYRange(bool v)
{
  fixYRange_ = v;
}

bool QProfileGraph::fixYRange() const
{
  return fixYRange_;
}

void QProfileGraph::setSkipZeroPixels(bool v)
{
  skipZeroPixels_ = v;
  Q_EMIT skipZeroPixlelsChanged();
}

bool QProfileGraph::skipZeroPixels() const
{
  return skipZeroPixels_;
}

void QProfileGraph::setXRangeMin(double v)
{
  plot_->xAxis->setRangeLower(v);
}

double QProfileGraph::xRangeMin() const
{
  return plot_->xAxis->range().lower;
}

void QProfileGraph::setXRangeMax(double v)
{
  plot_->xAxis->setRangeUpper(v);
}

double QProfileGraph::xRangeMax() const
{
  return plot_->xAxis->range().upper;
}

void QProfileGraph::setYRangeMin(double v)
{
  plot_->yAxis->setRangeLower(v);
}

double QProfileGraph::yRangeMin() const
{
  return plot_->yAxis->range().lower;
}

void QProfileGraph::setYRangeMax(double v)
{
  plot_->yAxis->setRangeUpper(v);
}

double QProfileGraph::yRangeMax() const
{
  return plot_->yAxis->range().upper;
}

void QProfileGraph::showProfilePlot(const QLineF & line, const cv::Mat & image)
{
  return showProfilePlot(QLine((int) line.x1(), (int) line.y1(), (int) line.x2(), (int) line.y2()), image);
}

void QProfileGraph::showProfilePlot(const QLine & line, const cv::Mat & image)
{
  QVector<cv::Point> pts;

  for( int c = 0; c < 4; ++c ) {
    current_keys_[c].clear();
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


    get_pixels(image, pts, skipZeroPixels_,
        current_keys_, current_values_);
  }

  const int cn =
      image.channels();

  for( int c = 0; c < 4; ++c ) {

    if( c < cn ) {
      graphs_[c]->setData(skipZeroPixels_ ?
          current_keys_[c] : current_keys_[0],
          current_values_[c]);
    }
    else {

      static const QVector<double> empty_keys;
      static const QVector<double> empty_values;

      graphs_[c]->setData(empty_keys, empty_values);
    }
  }

  if( !fixXRange_ ) {
    plot_->xAxis->setRange(0, pts.size());
    Q_EMIT xRangeRescaled();
  }

  if( !fixYRange_ ) {
    plot_->yAxis->rescale();
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
    if ( !current_keys_[c].empty() && !current_values_[c].isEmpty() ) {
      ++num_columns;
    }
  }

  if ( num_columns < 1 ) {
    CF_DEBUG("No Data available");
    return;
  }

  QClipboard * clipboard = QApplication::clipboard();
  if ( !clipboard ) {
    CF_DEBUG("No clipboard available");
    return;
  }


  QString text;

  if ( skipZeroPixels_ ) {

    text = "X0\tY0";

    for( int c = 1; c < num_columns; ++c ) {
      text.append(qsprintf("\tY%d", c));
    }

    text.append("\n");

    int max_points = current_keys_[0].size();
    for( int c = 1; c < num_columns; ++c ) {
      if( current_keys_[c].size() > max_points ) {
        max_points = current_keys_[c].size();
      }
    }

    for( int i = 0; i < max_points; ++i ) {
      for( int c = 0; c < num_columns; ++c ) {

        if( i >= current_keys_[c].size() ) {
          text.append(qsprintf(" \t "));
        }
        else {
          text.append(qsprintf("\t%g\t%g", current_keys_[c][i],
              current_values_[c][i]));
        }
      }

      text.append("\n");
    }

  }
  else {

    text = "X";

    for( int c = 0; c < num_columns; ++c ) {
      text.append(qsprintf("\tY%d", c));
    }

    text.append("\n");

    for ( int i = 0, n = current_keys_[0].size(); i < n; ++i ) {

      text.append(qsprintf("%g", current_keys_[0][i]));

      for( int c = 0; c < num_columns; ++c ) {
        text.append(qsprintf("\t%g", current_values_[c][i]));
      }

      text.append("\n");
    }
  }


  clipboard->setText(text);
  CF_DEBUG("Data copied");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphSettings::QProfileGraphSettings(QWidget * parent) :
    Base("QProfileGraphSettings", parent)
{
  lineStyle_ctl =
      add_enum_combobox<QCPGraph::LineStyle>("Line Style:",
          "Set line style",
          [this](QCPGraph::LineStyle v) {
            if ( profileGraph_ ) {
              profileGraph_->setLineStyle(v);
            }
          },
          [this](QCPGraph::LineStyle * v) {
            if ( profileGraph_ ) {
              *v = profileGraph_->lineStyle();
              return true;
            }
            return false;
          });

  fixXRange_ctl =
      add_checkbox("Fix X range:",
          "Set checked to fix X range of the plot",
          [this](bool checked) {
            if ( profileGraph_ ) {
              profileGraph_->setFixXRange(checked);
            }
          },
          [this](bool * checked) {
            if ( profileGraph_ ) {
              *checked = profileGraph_->fixXRange();
              return true;
            }
            return false;
          });


  xRangeMin_ctl =
      add_numeric_box<double>("Xmin:",
          "X range minimum value",
          [this](double v) {
            if ( profileGraph_ ) {
              profileGraph_->setXRangeMin(v);
              profileGraph_->replot();
            }
          },
          [this](double * v) {
            if ( profileGraph_ ) {
              *v = profileGraph_->xRangeMin();
              return true;
            }
            return false;
          });


  xRangeMax_ctl =
      add_numeric_box<double>("Xmax:",
          "X range maximum value",
          [this](double v) {
            if ( profileGraph_ ) {
              profileGraph_->setXRangeMax(v);
              profileGraph_->replot();
            }
          },
          [this](double * v) {
            if ( profileGraph_ ) {
              *v = profileGraph_->xRangeMax();
              return true;
            }
            return false;
          });

  ///

  fixYRange_ctl =
      add_checkbox("Fix Y range:",
          "Set checked to fix Y range of the plot",
          [this](bool checked) {
            if ( profileGraph_ ) {
              profileGraph_->setFixYRange(checked);
            }
          },
          [this](bool * checked) {
            if ( profileGraph_ ) {
              *checked = profileGraph_->fixYRange();
              return true;
            }
            return false;
          });


  yRangeMin_ctl =
      add_numeric_box<double>("Ymin:",
          "Y range minimum value",
          [this](double v) {
            if ( profileGraph_ ) {
              profileGraph_->setYRangeMin(v);
              profileGraph_->replot();
            }
          },
          [this](double * v) {
            if ( profileGraph_ ) {
              *v = profileGraph_->yRangeMin();
              return true;
            }
            return false;
          });


  yRangeMax_ctl =
      add_numeric_box<double>("Ymax:",
          "Y range maximum value",
          [this](double v) {
            if ( profileGraph_ ) {
              profileGraph_->setYRangeMax(v);
              profileGraph_->replot();
            }
          },
          [this](double * v) {
            if ( profileGraph_ ) {
              *v = profileGraph_->yRangeMax();
              return true;
            }
            return false;
          });

  ///

  skipZeros_ctl =
      add_checkbox("Skip Zero values:",
          "Set checked to skip zero valued values",
          [this](bool checked) {
            if ( profileGraph_ ) {
              profileGraph_->setSkipZeroPixels(checked);
            }
          },
          [this](bool * checked) {
            if ( profileGraph_ ) {
              *checked = profileGraph_->skipZeroPixels();
              return true;
            }
            return false;
          });

  ///

  updateControls();
}

void QProfileGraphSettings::setProfileGraph(QProfileGraph * profileGraph)
{
  if ( profileGraph_ ) {
    profileGraph_->disconnect(this);
  }

  if ( (profileGraph_ = profileGraph) ) {

    connect(profileGraph_, &QProfileGraph::xRangeRescaled,
        [this]() {
          c_update_controls_lock lock(this);
          xRangeMin_ctl->setValue(profileGraph_->xRangeMin());
          xRangeMax_ctl->setValue(profileGraph_->xRangeMax());
        });

    connect(profileGraph_, &QProfileGraph::yRangeRescaled,
        [this]() {
          c_update_controls_lock lock(this);
          yRangeMin_ctl->setValue(profileGraph_->yRangeMin());
          yRangeMax_ctl->setValue(profileGraph_->yRangeMax());
        });

    connect(profileGraph_, &QProfileGraph::skipZeroPixlelsChanged,
        this, &ThisClass::parameterChanged);

  }

  updateControls();
}

QProfileGraph * QProfileGraphSettings::profileGraph() const
{
  return profileGraph_;
}

void QProfileGraphSettings::onupdatecontrols()
{
  if ( !profileGraph_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

void QProfileGraphSettings::onload(QSettings & settings)
{
  // Base::onload(settings);
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


