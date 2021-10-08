/*
 * QMtfControl.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QMtfControl.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/create_histogram.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/debug.h>

#define ICON_histogram                "histogram"
#define ICON_histogram_linear_scale   "histogram-linear-scale2"
#define ICON_histogram_log_scale      "histogram-log-scale2"
#define ICON_histogram_automtf        "histogram-automtf"
#define ICON_reset                    "reset"
#define ICON_contrast                 "contrast"
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
static QIcon auto_clip_icon;
static QIcon auto_mtf_icon;

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
  if ( auto_clip_icon.isNull() ) {
    auto_clip_icon = getIcon(ICON_contrast);
  }
  if ( auto_mtf_icon.isNull() ) {
    auto_mtf_icon = getIcon(ICON_histogram_automtf);
  }
}



/** @brief Return default pixel values range for given image depth
 */
//static bool suggest_levels_range(int depth, double * minval, double * maxval)
//{
//  switch ( depth ) {
//  case CV_8U :
//    *minval = 0;
//    *maxval = UINT8_MAX;
//    break;
//  case CV_8S :
//    *minval = INT8_MIN;
//    *maxval = INT8_MAX;
//    break;
//  case CV_16U :
//    *minval = 0;
//    *maxval = UINT16_MAX;
//    break;
//  case CV_16S :
//    *minval = INT16_MIN;
//    *maxval = INT16_MAX;
//    break;
//  case CV_32S :
//    *minval = INT32_MIN;
//    *maxval = INT32_MAX;
//    break;
//  case CV_32F :
//    *minval = 0;
//    *maxval = 1;
//    break;
//  case CV_64F :
//    *minval = 0;
//    *maxval = 1;
//    break;
//  default:
//    *minval = 0;
//    *maxval = 1;
//    return false;
//  }
//
//  return true;
//}

/** @brief Return default pixel values range for given image depth
 */
static bool suggest_levels_range(cv::InputArray src, double * minval, double * maxval)
{
  switch (src.depth()) {
    case CV_32F:
    case CV_64F:
      cv::minMaxLoc(src, minval, maxval);
      break;
    default:
      return get_data_range_for_pixel_depth(src.depth(), minval, maxval);
  }

  return true;
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


  displayChannel_ctl = new QComboBox(this);
  displayChannel_ctl->setEditable(false);
  displayChannel_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  displayChannel_ctl->addItem("RGB", (int)QHistogramView::DisplayChannel_RGB);
  displayChannel_ctl->addItem("Grayscale", (int)(QHistogramView::DisplayChannel_Grayscale));
  displayChannel_ctl->addItem("Value", (int)(QHistogramView::DisplayChannel_Value));
  connect(displayChannel_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onDisplayChannelComboCurrentIndexChanged(int)) );
  topToolbar_->addWidget(displayChannel_ctl);

  addStretch(topToolbar_);


  chartTypeSelectorButton_ = new QToolButton();
  chartTypeSelectorButton_->setText("Chart type");
  chartTypeSelectorButton_->setToolTip("Select chart type");
  topToolbar_->addWidget(chartTypeSelectorButton_);

  connect(chartTypeSelectorButton_, &QToolButton::clicked,
      this, &ThisClass::onChartTypeSelectorClicked );


  //
  resetMtfAction_ = topToolbar_->addAction(getIcon(ICON_reset), "Reset Clippig");
  resetMtfAction_->setToolTip("Reset MTF clipping");
  connect(resetMtfAction_, &QAction::triggered,
      this, &ThisClass::onResetMtfClicked);


  //
  autoMtf_ctl = new QToolButton(this);
  autoMtf_ctl->setIconSize(QSize(16, 16));
  autoMtf_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  autoMtf_ctl->setCheckable(true);
  autoMtf_ctl->setText("Auto MTF");
  autoMtf_ctl->setToolTip("Auto MTF adjustment");
  topToolbar_->addWidget(autoMtf_ctl);

  connect(autoMtf_ctl, &QToolButton::clicked,
      this, &ThisClass::onAutoMtfCtrlClicked );

  autoMtfMenu.addAction(auto_clip_icon,
      "Auto clip",
      [this]() {
        selectedAutoMtfAction_ = AutoMtfAction_AutoClip;
        onAutoMtfCtrlClicked();
      });

  autoMtfMenu.addAction(auto_mtf_icon,
      "Auto Mtf",
      [this]() {
        selectedAutoMtfAction_ = AutoMtfAction_AutoMtf;
        onAutoMtfCtrlClicked();
      });

  autoMtf_ctl->setPopupMode(QToolButton::MenuButtonPopup);
  autoMtf_ctl->setMenu(&autoMtfMenu);


//  //
//
//  autoClipAction_ = topToolbar_->addAction(getIcon(ICON_contrast), "Auto Clip");
//  autoClipAction_->setToolTip("Auto clip image range");
//  connect(autoClipAction_, &QAction::triggered,
//      this, &ThisClass::onAutoClipClicked);
//
//
//  //
//
//  autoMtfAction_ = topToolbar_->addAction(getIcon(ICON_histogram_automtf), "Auto MTF");
//  autoMtfAction_->setToolTip("Find automatic midtones balance");
//  connect(autoMtfAction_, &QAction::triggered,
//      this, &ThisClass::onFindAutoMidtonesBalanceClicked);
//
//  //
//


  logScaleSelectionAction_ = topToolbar_->addAction(log_scale_icon, "Log scale");
  logScaleSelectionAction_ ->setToolTip("Switch between linear / log scale");
  logScaleSelectionAction_->setCheckable(true);
  logScaleSelectionAction_->setChecked(false);

  // configure gistogram view
  levelsView_ = new QHistogramView(this);
  logScaleSelectionAction_->setChecked(levelsView_->logScale());
  chartTypeSelectorButton_->setIcon(selectChartTypeIcon(levelsView_->chartType()) );
  displayChannel_ctl->setCurrentIndex(displayChannel_ctl->findData((int)levelsView_->displayChannel()));

  // configure mtf slider
  mtfSlider_ = new QMtfSlider(this);

  // configure toolbar2
  bottomToolbar_ = new QToolBar(this);
  bottomToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  bottomToolbar_->setIconSize(QSize(16, 16));

  for ( int i = 0; i < 3; ++i ) {
    bottomToolbar_->addWidget(spins[i] = new QDoubleSpinBox());
    spins[i]->setKeyboardTracking(false);
    spins[i]->setRange(0, 1);
    spins[i]->setDecimals(3);
    spins[i]->setSingleStep(0.01);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    spins[i]->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
#endif
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
      levelsView_, &QHistogramView::setLogScale);

  connect(mtfSlider_, &QMtfSlider::mtfChanged,
      [this]() {
        if ( mtf_ && !updatingControls_ ) {

          const bool wasInUpdatingControls =
              updatingControls();

          setUpdatingControls(true);

          mtf_->set_shadows(mtfSlider_->shadows());
          mtf_->set_highlights(mtfSlider_->highlights());
          mtf_->set_midtones(mtfSlider_->midtones());

          spins[SPIN_SHADOWS]->setValue(mtf_->shadows());
          spins[SPIN_HIGHLIGHTS]->setValue(mtf_->highlights());
          spins[SPIN_MIDTONES]->setValue(mtf_->midtones());

          setUpdatingControls(wasInUpdatingControls);
          if ( !wasInUpdatingControls ) {
            emit mtfChanged();
          }
        }
      });

  connect(spins[SPIN_SHADOWS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if (  mtf_ && !updatingControls_ && v != mtf_->shadows() ) {

          const bool wasInUpdatingControls =
              updatingControls();

          setUpdatingControls(true);

          mtf_->set_shadows(v);
          mtfSlider_->setShadows(mtf_->shadows());

          setUpdatingControls(wasInUpdatingControls);
          if ( !wasInUpdatingControls ) {
            emit mtfChanged();
          }
        }
      });

  connect(spins[SPIN_HIGHLIGHTS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( mtf_ && !updatingControls_ && v != mtf_->highlights() ) {

          const bool wasInUpdatingControls =
              updatingControls();

          setUpdatingControls(true);

          mtf_->set_highlights(v);
          mtfSlider_->setHighlights(mtf_->highlights());

          setUpdatingControls(wasInUpdatingControls);
          if ( !wasInUpdatingControls ) {
            emit mtfChanged();
          }
        }
      });

  connect(spins[SPIN_MIDTONES], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( mtf_ && !updatingControls_ && v != mtf_->midtones() ) {

          const bool wasInUpdatingControls =
              updatingControls();

          setUpdatingControls(true);

          mtf_->set_midtones(v);
          mtfSlider_->setMidtones(mtf_->midtones());

          setUpdatingControls(wasInUpdatingControls);
          if ( !wasInUpdatingControls ) {
            emit mtfChanged();
          }
        }
      });

  updateControls();
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

bool QMtfControl::updatingControls() const
{
  return updatingControls_;
}

void QMtfControl::setUpdatingControls(bool v)
{
  updatingControls_ = v;
}


void QMtfControl::updateControls()
{
  if ( !mtf_ ) {
    setEnabled(false);
  }
  else {

    const bool wasInUpdatingControls =
        updatingControls();

    setUpdatingControls(true);

    mtfSlider_->setup(mtf_->shadows(), mtf_->highlights(), mtf_->midtones());

    spins[SPIN_SHADOWS]->setValue(mtf_->shadows());
    spins[SPIN_HIGHLIGHTS]->setValue(mtf_->highlights());
    spins[SPIN_MIDTONES]->setValue(mtf_->midtones());
    logScaleSelectionAction_->setChecked(levelsView_->logScale());

    const bool autoMtfChecked =
        autoMtf_ctl->isChecked();

    spins[SPIN_SHADOWS]->setEnabled(!autoMtfChecked);
    spins[SPIN_HIGHLIGHTS]->setEnabled(!autoMtfChecked);
    spins[SPIN_MIDTONES]->setEnabled(!autoMtfChecked);
    mtfSlider_->setEnabled(!autoMtfChecked);

    switch ( selectedAutoMtfAction_ ) {
    case AutoMtfAction_AutoMtf :
      autoMtf_ctl->setIcon(auto_mtf_icon);
      break;
    case AutoMtfAction_AutoClip :
      default :
      autoMtf_ctl->setIcon(auto_clip_icon);
      break;
    }

    setUpdatingControls(wasInUpdatingControls);

    setEnabled(true);
  }

}

void QMtfControl::setInputImage(cv::InputArray image, cv::InputArray mask)
{
  inputImage_ = image.getMat();
  inputMask_ = mask.getMat();
  onAutoMtfCtrlClicked();
}

void QMtfControl::setOutputImage(cv::InputArray image, cv::InputArray mask)
{
  levelsView_->setImage(image, mask);
}


void QMtfControl::onResetMtfClicked()
{
  QWaitCursor wait(this);

  const bool wasInUpdatingControls =
      updatingControls();

  setUpdatingControls(true);

  mtf_->set_shadows(0);
  mtf_->set_highlights(1);
  mtf_->set_midtones(0.5);

  updateControls();

  setUpdatingControls(wasInUpdatingControls);
  if ( !wasInUpdatingControls ) {
    emit mtfChanged();
  }
}


bool QMtfControl::isAutoMtfActionEnabled() const
{
  return autoMtf_ctl && autoMtf_ctl->isChecked();
}



void QMtfControl::onAutoMtfCtrlClicked()
{
  if ( autoMtf_ctl->isChecked() ) {

    switch ( selectedAutoMtfAction_ ) {
    case AutoMtfAction_AutoMtf :
      findAutoMidtonesBalance();
      break;

    case AutoMtfAction_AutoClip :
      default :
      findAutoHistogramClips();
      break;
    }
  }

  updateControls();

  if ( !updatingControls() ) {
    emit mtfChanged();
  }

}



void QMtfControl::findAutoHistogramClips()
{
  if( !inputImage_.empty() ) {

    QWaitCursor wait(this);

    double hmin = -1, hmax = -1;
    double minval = 0, maxval = 1;

    minmax(inputImage_, &hmin, &hmax, inputMask_);
    suggest_levels_range(inputImage_, &minval, &maxval);

    mtf_->set_shadows((hmin - minval) / (maxval - minval));
    mtf_->set_highlights((hmax - minval) / (maxval - minval));
    mtf_->set_midtones(0.5);

    //updateControls();

    if ( !updatingControls() ) {
      emit mtfChanged();
    }
  }
}

void QMtfControl::findAutoMidtonesBalance()
{
  if ( mtf_ && !inputImage_.empty() ) {

    QWaitCursor wait(this);

    cv::Mat1f H;
    double hmin = -1, hmax = -1;

    if ( !create_histogram(inputImage_, inputMask_, H, &hmin, &hmax) ) {
      CF_ERROR("create_image_histogram() fails");
    }
    else {
      mtf_->find_midtones_balance(H);

      //updateControls();

      if ( !updatingControls() ) {
        emit mtfChanged();
      }
    }
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

void QMtfControl::onDisplayChannelComboCurrentIndexChanged(int)
{
  levelsView_->setDisplayChannel((QHistogramView::DisplayChannel)
      displayChannel_ctl->currentData().toInt());
}

