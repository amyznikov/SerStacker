/*
 * QProfileGraph.cc
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#include "QProfileGraph.h"
#include <gui/widgets/QToolBarStretch.h>
#include <gui/widgets/style.h>


#define ICON_plot       ":/qmeasure/icons/plot.png"
#define ICON_settings   ":/qmeasure/icons/settings.png"
#define ICON_copy       ":/qmeasure/icons/copy.png"

namespace {

template<class T>
void get_pixels_(const cv::Mat & image, const QVector<cv::Point> & pts, QVector<double> & keys, QVector<double> values[4])
{
  const cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  keys.clear();

  for ( int i = 0; i < 4; ++i ) {
    values[i].clear();
  }

  for( int i = 0, n = pts.size(); i < n; ++i ) {

    const cv::Point & p =
        pts[i];

    if( p.x >= 0 && p.x < src.cols && p.y >= 0 && p.y < src.rows ) {

      keys.append(i);

      const T *srcp = src[p.y];

      for( int c = 0; c < cn; ++c ) {
        values[c].append(srcp[p.x * cn + c]);
      }

    }
  }

}

void get_pixels(const cv::Mat & image, const QVector<cv::Point> & pts, QVector<double> & keys, QVector<double> values[4])
{
  switch (image.depth()) {
    case CV_8U:
      get_pixels_<uint8_t>(image, pts, keys, values);
      break;
    case CV_8S:
      get_pixels_<int8_t>(image, pts, keys, values);
      break;
    case CV_16U:
      get_pixels_<uint16_t>(image, pts, keys, values);
      break;
    case CV_16S:
      get_pixels_<int16_t>(image, pts, keys, values);
      break;
    case CV_32S:
      get_pixels_<int32_t>(image, pts, keys, values);
      break;
    case CV_32F:
      get_pixels_<float>(image, pts, keys, values);
      break;
    case CV_64F:
      get_pixels_<double>(image, pts, keys, values);
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
  current_keys_.clear();
  for ( int i = 0; i < 4; ++i ) {
    current_values_[i].clear();
  }

  if ( &line != &currentLine_ ) {
    currentLine_ = line;
  }

  QVector<cv::Point> pts;

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

      if ( x2 >= x1 ) {

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

      if ( y2 >= y1  ) {

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


  const int cn =
      image.channels();

  get_pixels(image, pts, current_keys_, current_values_);

  for( int c = 0; c < 4; ++c ) {

    if ( c < cn ) {
      graphs_[c]->setData(current_keys_, current_values_[c]);
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
  if ( current_keys_.empty() ) {
    CF_DEBUG("No Data available");
    return;
  }

  int num_columns = 0;

  for ( int j = 0; j < 4; ++j ) {
    if ( !current_values_[j].isEmpty() ) {
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

  QString text = "X";

  for( int j = 0; j < num_columns; ++j ) {
    text.append(qsprintf("\tY%d", j));
  }

  text.append("\n");

  for ( int i = 0, n = current_keys_.size(); i < n; ++i ) {

    text.append(qsprintf("%g", current_keys_[i]));

    for( int j = 0; j < num_columns; ++j ) {
      text.append(qsprintf("\t%g", current_values_[j][i]));
    }

    text.append("\n");
  }

  clipboard->setText(text);

  CF_DEBUG("Data copied");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QProfileGraphSettings::QProfileGraphSettings(QWidget * parent) :
    Base("QProfileGraphSettings", parent)
{
  fixXRange_ctl =
      add_checkbox("Fix X range:",
          "Set checked to fix X range of the plot",
          [this](bool checked) {
            if ( profileGraph_ ) {
              profileGraph_->setFixXRange(checked);
//              xRangeMin_ctl->setEnabled(!checked);
//              xRangeMax_ctl->setEnabled(!checked);
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
//              yRangeMin_ctl->setEnabled(!checked);
//              yRangeMax_ctl->setEnabled(!checked);
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

//    xRangeMin_ctl->setEnabled(!profileGraph_->fixXRange());
//    xRangeMax_ctl->setEnabled(!profileGraph_->fixXRange());
//
//    yRangeMin_ctl->setEnabled(!profileGraph_->fixYRange());
//    yRangeMax_ctl->setEnabled(!profileGraph_->fixYRange());

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


