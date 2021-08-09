/*
 * QMtfControl.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QMtfControl.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/histogram/create_image_histogram.h>
#include <core/debug.h>

#define ICON_histogram                "histogram"
#define ICON_histogram_linear_scale   "histogram-linear-scale"
#define ICON_histogram_log_scale      "histogram-log-scale"
#define ICON_histogram_automtf        "histogram-automtf"
#define ICON_bar_chart                "bar_chart"
#define ICON_line_chart               "line_chart"



static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qmtfcontrols/icons/%1").arg(name));
}

static QPixmap getPixmap(const QString & name)
{
  return QPixmap(QString(":/qmtfcontrols/icons/%1").arg(name));
}

static QIcon log_scale_icon;
static QIcon bar_chart_icon;
static QIcon line_chart_icon;

static const QIcon & selectChartTypeIcon(QHistogramView::ChartType chartType)
{
  switch ( chartType ) {
  case QHistogramView::ChartType_Lines :
    return line_chart_icon;
  case QHistogramView::ChartType_Bars :
    default :
    break;
  }
  return bar_chart_icon;
}



static void addStretch(QToolBar * toolbar)
{
  QWidget* empty = new QWidget();
  empty->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(empty);
}

static void intit_mtfcontrols_resources()
{
  Q_INIT_RESOURCE(qmtfcontrols_resources);

  if ( bar_chart_icon.isNull() ) {
    bar_chart_icon = getIcon(ICON_bar_chart);
  }
  if ( line_chart_icon.isNull() ) {
    line_chart_icon = getIcon(ICON_line_chart);
  }
  if ( log_scale_icon.isNull() ) {
    log_scale_icon.addPixmap(getPixmap(ICON_histogram_linear_scale), QIcon::Normal, QIcon::Off);
    log_scale_icon.addPixmap(getPixmap(ICON_histogram_log_scale), QIcon::Normal, QIcon::On);
  }
}

QMtfControl::QMtfControl(QWidget * parent)
    : Base(parent)
{
  intit_mtfcontrols_resources();

  vbox_ = new QVBoxLayout(this);

  // configure toolbar1

  topToolbar_ = new QToolBar(this);
  topToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  topToolbar_->setIconSize(QSize(16, 16));

  addStretch(topToolbar_);


  chartTypeSelectorButton_ = new QToolButton();
  chartTypeSelectorButton_->setText("Chart type");
  chartTypeSelectorButton_->setToolTip("Select chart type");
  topToolbar_->addWidget(chartTypeSelectorButton_);

  connect(chartTypeSelectorButton_, &QToolButton::clicked,
      this, &ThisClass::onChartTypeSelectorClicked );



  //
  autoMtfAction_ = topToolbar_->addAction(getIcon(ICON_histogram_automtf), "Auto MTF");
  autoMtfAction_->setToolTip("Find automatic midtones balance");
  connect(autoMtfAction_, &QAction::triggered,
      this, &ThisClass::findAutoMidtonesBalance);

  //


  logScaleSelectionAction_ = topToolbar_->addAction(log_scale_icon, "Log scale");
  logScaleSelectionAction_->setCheckable(true);
  logScaleSelectionAction_->setChecked(false);

  // configure gistogram view
  levelsView_ = new QHistogramView(this);
  levelsView_->setLogScale(logScaleSelectionAction_->isChecked());
  chartTypeSelectorButton_->setIcon(selectChartTypeIcon(levelsView_->chartType()) );

  // configure mtf slider
  mtfSlider_ = new QMtfSlider(this);

  // configure toolbar2
  bottomToolbar_ = new QToolBar(this);
  bottomToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  bottomToolbar_->setIconSize(QSize(16, 16));

  for ( int i = 0; i < 3; ++i ) {
    bottomToolbar_->addWidget(spins[i] = new QDoubleSpinBox());
    spins[i]->setRange(0, 1);
    spins[i]->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
    spins[i]->setSingleStep(0.01);
    if ( i < 2 ) {
      addStretch(bottomToolbar_);
    }
  }

  //setupLevels(0, 256);

  // Combine layouts
  vbox_->addWidget(topToolbar_);
  vbox_->addWidget(levelsView_);
  vbox_->addWidget(mtfSlider_);
  vbox_->addWidget(bottomToolbar_);

  // Configure event handles

  connect(logScaleSelectionAction_, &QAction::triggered,
      [this]() {
        levelsView_->setLogScale(logScaleSelectionAction_->isChecked());
        levelsView_->update();
      });

  connect(mtfSlider_, &QMtfSlider::mtfChanged,
      [this]() {
        if ( mtf_ && !updatingControls_ ) {

          updatingControls_ = true;

          mtf_->set_shadows(mtfSlider_->shadows());
          mtf_->set_highlights(mtfSlider_->highlights());
          mtf_->set_midtones(mtfSlider_->midtones());

          spins[SPIN_SHADOWS]->setValue(mtf_->shadows());
          spins[SPIN_HIGHLIGHTS]->setValue(mtf_->highlights());
          spins[SPIN_MIDTONES]->setValue(mtf_->midtones());

          updateHistogramView();
          updatingControls_ = false;

          emit mtfChanged();
        }
      });

  connect(spins[SPIN_SHADOWS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if (  mtf_ && !updatingControls_ && v != mtf_->shadows() ) {
          updatingControls_ = true;
          mtf_->set_shadows(v);

          mtfSlider_->setShadows(mtf_->shadows());
          updateHistogramView();

          updatingControls_ = false;
          emit mtfChanged();
        }
      });

  connect(spins[SPIN_HIGHLIGHTS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( mtf_ && !updatingControls_ && v != mtf_->highlights() ) {
          updatingControls_ = true;
          mtf_->set_highlights(v);

          mtfSlider_->setHighlights(mtf_->highlights());
          updateHistogramView();

          updatingControls_ = false;
          emit mtfChanged();
        }
      });

  connect(spins[SPIN_MIDTONES], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( mtf_ && !updatingControls_ && v != mtf_->midtones() ) {
          updatingControls_ = true;
          mtf_->set_midtones(v);

          mtfSlider_->setMidtones(mtf_->midtones());
          updateHistogramView();

          updatingControls_ = false;
          emit mtfChanged();
        }
      });

}


void QMtfControl::setMtf(const c_pixinsight_midtones_transfer_function::ptr & mtf)
{
  this->mtf_ = mtf;
  updateControls();
}

const c_pixinsight_midtones_transfer_function::ptr & QMtfControl::mtf() const
{
  return this->mtf_;
}


void QMtfControl::updateControls()
{
  if ( !mtf_ ) {
    setEnabled(false);
  }
  else {

    updatingControls_ = true;

    mtfSlider_->setup(mtf_->shadows(), mtf_->highlights(), mtf_->midtones());
    spins[SPIN_SHADOWS]->setValue(mtf_->shadows());
    spins[SPIN_HIGHLIGHTS]->setValue(mtf_->highlights());
    spins[SPIN_MIDTONES]->setValue(mtf_->midtones());
    logScaleSelectionAction_->setChecked(levelsView_->logScale());

    updatingControls_ = false;

    setEnabled(true);
  }

}

void QMtfControl::updateHistogramView()
{
  int first_bin, last_bin;

  if ( mtf_ ) {
    first_bin = imageHistogram.rows * mtf_->shadows();
    last_bin = (imageHistogram.rows - 1) * mtf_->highlights();
  }
  else {
    first_bin = 0;
    last_bin = imageHistogram.rows - 1;
  }

  levelsView_->
      showHistogram(imageHistogram, -1,
          first_bin, 0,
          last_bin, 0);

}

void QMtfControl::setImage(cv::InputArray input_image)
{
  levelsView_->clear();

  int nbins = input_image.depth() > CV_8S ? 65536 : 256;


  create_image_histogram(input_image,
      imageHistogram,
      nbins,
      -1);

  updateHistogramView();
}

void QMtfControl::findAutoMidtonesBalance()
{
  if ( mtf_ && !imageHistogram.empty() ) {
    QWaitCursor wait(this);
    mtf_->find_midtones_balance(imageHistogram);
    updateControls();
    emit mtfChanged();
  }
}

void QMtfControl::onChartTypeSelectorClicked()
{
  static QMenu chartTypesMenu;
  static QAction * setLineChartAction = Q_NULLPTR;
  static QAction * setBarChartAction = Q_NULLPTR;

  if ( chartTypesMenu.isEmpty() ) {
    chartTypesMenu.addAction(setLineChartAction = new QAction(line_chart_icon, "Lines"));
    chartTypesMenu.addAction(setBarChartAction = new QAction(bar_chart_icon, "Bars"));
  }

  QAction * selectedAction =
      chartTypesMenu.exec(chartTypeSelectorButton_->mapToGlobal(QPoint(
          2 * chartTypeSelectorButton_->width() / 3,
          chartTypeSelectorButton_->height() / 2)));

  if ( selectedAction ) {

    if ( selectedAction == setLineChartAction ) {
      levelsView_->setChartType(QHistogramView::ChartType_Lines);
    }
    else if ( selectedAction == setBarChartAction ) {
      levelsView_->setChartType(QHistogramView::ChartType_Bars);
    }

    chartTypeSelectorButton_->setIcon(selectChartTypeIcon(
        levelsView_->chartType()));
  }

}
