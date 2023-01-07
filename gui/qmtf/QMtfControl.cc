/*
 * QMtfDisplaySettings.cc
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <gui/widgets/settings.h>
#include <gui/qimageview/cv2qt.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/debug.h>
#include "QMtfControl.h"


#define ICON_histogram                ":/qmtf/icons/histogram"
#define ICON_histogram_linear_scale   ":/qmtf/icons/histogram-linear-scale2"
#define ICON_histogram_log_scale      ":/qmtf/icons/histogram-log-scale2"
#define ICON_histogram_automtf        ":/qmtf/icons/histogram-automtf"
#define ICON_reset                    ":/qmtf/icons/reset"
#define ICON_contrast                 ":/qmtf/icons/contrast"
#define ICON_bar_chart                ":/qmtf/icons/bar_chart"
#define ICON_line_chart               ":/qmtf/icons/line_chart"
#define ICON_colormap                 ":/qmtf/icons/colormap2"

static QIcon log_scale_icon;
static QIcon bar_chart_icon;
static QIcon line_chart_icon;
static QIcon auto_clip_icon;
static QIcon auto_mtf_icon;
static QIcon colormap_icon;

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

static void intit_mtfcontrol_resources()
{
  Q_INIT_RESOURCE(qmtf_resources);

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
  if ( colormap_icon.isNull() ) {
    colormap_icon = getIcon(ICON_colormap);
  }

}


/////////////////////////////////////////////////////////////////////////////////////////

QMtfControl::QMtfControl(QWidget * parent) :
    Base(parent)
{
  intit_mtfcontrol_resources();

  vbox_ = new QVBoxLayout(this);

  topToolbar_ctl = new QToolBar(this);
  topToolbar_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  topToolbar_ctl->setIconSize(QSize(16, 16));


  displayType_ctl = new QComboBox(this);
  displayType_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  displayType_ctl->setToolTip("Select data to visualize");
  topToolbar_ctl->addWidget(displayType_ctl);

  connect(displayType_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onDisplayTypeCurrentItemChanged);

  inputDataRange_ctl = new QNumberEditBox(this);
  inputDataRange_ctl->setToolTip("Set input data range (min/max clips)");
  topToolbar_ctl->addWidget(inputDataRange_ctl);
  connect(inputDataRange_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onInputDataRangeChanged);


  resetMtfAction_ = topToolbar_ctl->addAction(getIcon(ICON_reset), "Reset");
  resetMtfAction_->setToolTip("Reset MTF clipping and recompute input data range");
  connect(resetMtfAction_, &QAction::triggered,
      this, &ThisClass::onResetMtfClicked);


  //
  addStretch(topToolbar_ctl);

  colormap_ctl = new QToolButton();
  colormap_ctl->setIcon(colormap_icon);
  colormap_ctl->setText("Colormap");
  colormap_ctl->setToolTip("Select colormap");
  topToolbar_ctl->addWidget(colormap_ctl);
  connect(colormap_ctl, &QToolButton::clicked,
      this, &ThisClass::onColormapCtlClicked);


  chartTypeSelectorButton_ = new QToolButton();
  chartTypeSelectorButton_->setText("Chart type");
  chartTypeSelectorButton_->setToolTip("Select chart type");
  topToolbar_ctl->addWidget(chartTypeSelectorButton_);
  connect(chartTypeSelectorButton_, &QToolButton::clicked,
      this, &ThisClass::onChartTypeSelectorClicked );




  //
  autoMtf_ctl = new QToolButton(this);
  autoMtf_ctl->setIconSize(QSize(16, 16));
  autoMtf_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  autoMtf_ctl->setCheckable(true);
  autoMtf_ctl->setText("Auto MTF");
  autoMtf_ctl->setToolTip("Auto MTF adjustment");
  topToolbar_ctl->addWidget(autoMtf_ctl);
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

  logScaleSelectionAction_ = topToolbar_ctl->addAction(log_scale_icon, "Log scale");
  logScaleSelectionAction_ ->setToolTip("Switch between linear / log scale");
  logScaleSelectionAction_->setCheckable(true);
  logScaleSelectionAction_->setChecked(false);

  // configure gistogram view
  levelsView_ctl = new QHistogramView(this);
  logScaleSelectionAction_->setChecked(levelsView_ctl->logScale());
  chartTypeSelectorButton_->setIcon(selectChartTypeIcon(levelsView_ctl->chartType()) );
  //displayChannel_ctl->setCurrentIndex(displayChannel_ctl->findData((int)levelsView_->displayChannel()));

  // configure mtf slider
  mtfSlider_ctl = new QMtfSlider(this);
  colormap_strip_ctl = new QLabel(this);
  //  colormap_strip_ctl->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

  // configure toolbar2
  bottomToolbar_ctl = new QToolBar(this);
  bottomToolbar_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  bottomToolbar_ctl->setIconSize(QSize(16, 16));

  for ( int i = 0; i < 3; ++i ) {
    bottomToolbar_ctl->addWidget(spins[i] = new QDoubleSpinBox());
    spins[i]->setKeyboardTracking(false);
    spins[i]->setRange(0, 1);
    spins[i]->setDecimals(3);
    spins[i]->setSingleStep(0.01);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    spins[i]->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
#endif
    if ( i < 2 ) {
      addStretch(bottomToolbar_ctl);
    }
  }

  // Combine layouts
  vbox_->addWidget(topToolbar_ctl, 1);
  vbox_->addWidget(levelsView_ctl, 1000);
  vbox_->addWidget(mtfSlider_ctl, 1);
  vbox_->addWidget(colormap_strip_ctl, 1);
  vbox_->addWidget(bottomToolbar_ctl, 1);

  // Configure event handles

  connect(logScaleSelectionAction_, &QAction::triggered,
      levelsView_ctl, &QHistogramView::setLogScale);

  connect(mtfSlider_ctl, &QMtfSlider::mtfChanged,
      [this]() {
        if ( !updatingControls_ ) {

          if ( displaySettings_ ) {

            c_pixinsight_mtf & mtf =
                displaySettings_->mtf();

            const bool wasInUpdatingControls =
                updatingControls();

            setUpdatingControls(true);

            mtf.set_shadows(mtfSlider_ctl->shadows());
            mtf.set_highlights(mtfSlider_ctl->highlights());
            mtf.set_midtones(mtfSlider_ctl->midtones());

            spins[SPIN_SHADOWS]->setValue(mtf.shadows());
            spins[SPIN_HIGHLIGHTS]->setValue(mtf.highlights());
            spins[SPIN_MIDTONES]->setValue(mtf.midtones());

            setUpdatingControls(wasInUpdatingControls);

            if ( !wasInUpdatingControls ) {
              updateHistogramLevels();
              displaySettings_->updateDisplay();
            }
          }
        }
      });

  connect(spins[SPIN_SHADOWS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( !updatingControls_ ) {

          if ( displaySettings_ ) {

            c_pixinsight_mtf & mtf =
                displaySettings_->mtf();

            if ( v != mtf.shadows() ) {

              const bool wasInUpdatingControls =
                  updatingControls();

              setUpdatingControls(true);

              mtf.set_shadows(v);
              mtfSlider_ctl->setShadows(mtf.shadows());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit displaySettings_->updateDisplay();
              }
            }
          }
        }
      });

  connect(spins[SPIN_HIGHLIGHTS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {

        if ( !updatingControls_ ) {

          if ( displaySettings_ ) {

            c_pixinsight_mtf & mtf =
                displaySettings_->mtf();

            if ( v != mtf.highlights() ) {

              const bool wasInUpdatingControls =
                  updatingControls();

              setUpdatingControls(true);

              mtf.set_highlights(v);
              mtfSlider_ctl->setHighlights(mtf.highlights());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit displaySettings_->updateDisplay();
              }
            }
          }
        }
      });

  connect(spins[SPIN_MIDTONES], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( !updatingControls_ ) {

          if ( displaySettings_ ) {

            c_pixinsight_mtf & mtf =
                displaySettings_->mtf();

            if ( v != mtf.midtones() ) {

              const bool wasInUpdatingControls =
                  updatingControls();

              setUpdatingControls(true);

              mtf.set_midtones(v);
              mtfSlider_ctl->setMidtones(mtf.midtones());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit displaySettings_->updateDisplay();
              }
            }
          }
        }
      });

  updateControls();
}

void QMtfControl::setDisplaySettings(QMtfDisplay * displaySettings)
{
  const bool wasInUpdatingControls =
      updatingControls();

  setUpdatingControls(true);

  if( displaySettings_ ) {
    displaySettings_->disconnect(this);
  }

  displaySettings_ = displaySettings;
  displayType_ctl->clear();

  if( displaySettings_ ) {

    const c_enum_member * displayType =
        displaySettings_->displayTypes();

    for ( ; displayType->name && *displayType->name; ++ displayType ) {

      displayType_ctl->addItem(displayType->name,
          displayType->value);

    }

    connect(displaySettings_, &QMtfDisplay::inputDataChanged,
        this, &ThisClass::updateHistogramLevels,
        Qt::QueuedConnection);
  }

  setUpdatingControls(wasInUpdatingControls);

  updateControls();
}

QMtfDisplay * QMtfControl::displaySettings() const
{
  return displaySettings_;
}

bool QMtfControl::isAutoMtfActionEnabled() const
{
  return autoMtf_ctl && autoMtf_ctl->isChecked();
}

bool QMtfControl::updatingControls() const
{
  return updatingControls_;
}

void QMtfControl::setUpdatingControls(bool v)
{
  updatingControls_ = v;
}

void QMtfControl::onChartTypeSelectorClicked()
{
  static QMenu chartTypesMenu;
  static QAction *setLineChartAction = Q_NULLPTR;
  static QAction *setBarChartAction = Q_NULLPTR;

  if( chartTypesMenu.isEmpty() ) {
    chartTypesMenu.addAction(setLineChartAction = new QAction(line_chart_icon, "Lines"));
    chartTypesMenu.addAction(setBarChartAction = new QAction(bar_chart_icon, "Bars"));
  }

  QAction *selectedAction =
      chartTypesMenu.exec(chartTypeSelectorButton_->mapToGlobal(QPoint(
          2 * chartTypeSelectorButton_->width() / 3,
          chartTypeSelectorButton_->height() / 2)));

  if( selectedAction ) {

    if( selectedAction == setLineChartAction ) {
      levelsView_ctl->setChartType(QHistogramView::ChartType_Lines);
    }
    else if( selectedAction == setBarChartAction ) {
      levelsView_ctl->setChartType(QHistogramView::ChartType_Bars);
    }

    chartTypeSelectorButton_->setIcon(selectChartTypeIcon(
        levelsView_ctl->chartType()));
  }
}

void QMtfControl::onResetMtfClicked()
{
  if ( displaySettings_ ) {

    QWaitCursor wait(this);


    const bool wasInUpdatingControls =
        updatingControls();

    setUpdatingControls(true);

    double min = -1, max = -1;

    c_pixinsight_mtf & mtf =
        displaySettings_->mtf();

    mtf.get_input_range(&min, &max);
    if( min < max ) {
      min = max = -1;
    }
    else {
      displaySettings_->getInputDataRange(&min, &max);
    }

    mtf.set_input_range(min, max);
    mtf.set_shadows(0);
    mtf.set_highlights(1);
    mtf.set_midtones(0.5);
    displaySettings_->saveParameters();

    updateControls();

    setUpdatingControls(wasInUpdatingControls);
    if ( !wasInUpdatingControls ) {
      //updateHistogramLevels();
      emit displaySettings_->updateDisplay();
    }
  }
}

void QMtfControl::onAutoMtfCtrlClicked()
{
  if ( displaySettings_ ) {

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
      //updateHistogramLevels();
      emit displaySettings_->updateDisplay();
    }
  }
}

void QMtfControl::onColormapCtlClicked()
{
  if( displaySettings_ ) {

    QMenu menu;
    QAction * invertColormapAction;

    const COLORMAP current_colormap =
        displaySettings_->colormap();


    menu.addAction(invertColormapAction = new QAction("Invert colormap"));
    invertColormapAction->setCheckable(true);
    invertColormapAction->setChecked(displaySettings_->invertColormap());
    menu.addSeparator();

    for( const c_enum_member *colormap = members_of<COLORMAP>(); colormap->name; ++colormap ) {

      QAction * action = new QAction(colormap->name);
      action->setData((int) (colormap->value));

      if( colormap->value == current_colormap ) {
        action->setCheckable(true);
        action->setChecked(true);
      }

      menu.addAction(action);
    }



    QAction *action =
        menu.exec(colormap_ctl->mapToGlobal(
            QPoint(0, 0)));

    if ( action ) {

      if( action == invertColormapAction ) {
        displaySettings_->setInvertColormap(invertColormapAction->isChecked());
      }
      else {
        displaySettings_->setColormap((COLORMAP) action->data().toInt());
      }

      displaySettings_->saveParameters();

      updateHistogramLevels();
      updateColormapPixmap();
    }
  }
}

void QMtfControl::onDisplayTypeCurrentItemChanged()
{
  if( displaySettings_ && !updatingControls_ ) {
    displaySettings_->setDisplayType(displayType_ctl->currentData().toInt());
    displaySettings_->saveParameters();
    updateControls();
  }
}

void QMtfControl::onInputDataRangeChanged()
{
  if( displaySettings_ && !updatingControls_ ) {

    double range[2];

    if( fromString(inputDataRange_ctl->text(), range, 2) == 2 ) {

      c_pixinsight_mtf &mtf =
          displaySettings_->mtf();

      const bool wasInUpdatingControls =
          updatingControls();

      setUpdatingControls(true);

      mtf.set_input_range(range[0], range[1]);
      displaySettings_->saveParameters();

      setUpdatingControls(wasInUpdatingControls);

      if( !wasInUpdatingControls ) {
        updateHistogramLevels();
        emit displaySettings_->updateDisplay();
      }
    }
  }
}



void QMtfControl::findAutoHistogramClips()
{
  if( displaySettings_ ) {

    QWaitCursor wait(this);

    c_pixinsight_mtf &mtf =
        displaySettings_->mtf();

    double data_min = -1, data_max = -1;
    double range_min = -1, range_max = -1;

    displaySettings_->getInputDataRange(&data_min, &data_max);
    mtf.get_input_range(&range_min, &range_max);

    if ( data_min >= data_max ) {
      data_min = 0;
      data_max = 1;
    }

    if ( range_min >= range_max ) {
      range_min = data_min;
      range_max = data_max;
      //c_midtones_transfer_function::suggest_levels_range(depth, minval, maxval)
    }

    mtf.set_shadows((data_min - range_min) / (range_max - range_min));
    mtf.set_highlights((data_max - range_min) / (range_max - range_min));
    mtf.set_midtones(0.5);

    if( !updatingControls() ) {
      emit displaySettings_->updateDisplay();
    }
  }
}

void QMtfControl::findAutoMidtonesBalance()
{
  if( displaySettings_ ) {

    QWaitCursor wait(this);

    c_pixinsight_mtf &mtf =
        displaySettings_->mtf();

    cv::Mat1f H;
    double hmin = -1, hmax = -1;

    displaySettings_->getInputHistogramm(H, &hmin, &hmax);
    if( H.empty() ) {
      CF_ERROR("currentDisplay_->create_input_histogramm() fails");
    }
    else {
      mtf.find_midtones_balance(H);

      updateControls();

      if( !updatingControls() ) {
        emit displaySettings_->updateDisplay();
      }
    }
  }

}

void QMtfControl::updateHistogramLevels()
{
  if ( isVisible() ) {

    cv::Mat1f H;
    double hmin = -1, hmax = -1;

    if( displaySettings_ ) {
      // QWaitCursor wait(this);
      displaySettings_->getOutputHistogramm(H, &hmin, &hmax);
    }

    levelsView_ctl->setHistogram(H, hmin, hmax);
  }
}

void QMtfControl::updateColormapPixmap()
{
  if ( displaySettings_ ) {

    cv::Mat1b gray_image(16, 256);
    QImage qimage;

    for( int y = 0; y < gray_image.rows; ++y ) {
      for( int x = 0; x < gray_image.cols; ++x ) {
        gray_image[y][x] = x;
      }
    }

    const COLORMAP cmap =
        displaySettings_->colormap();

    if ( cmap == COLORMAP_NONE ) {
      cv2qt(gray_image, &qimage);
    }
    else {
      cv::Mat3b color_image;
      apply_colormap(gray_image, color_image, cmap);
      cv2qt(color_image, &qimage);
    }

    if ( displaySettings_->invertColormap() ) {

      qimage = qimage.mirrored(true, false);
    }

    colormap_pixmap_ =
        QPixmap::fromImage(qimage);

  }

  updateColormapStrip();

}

void QMtfControl::updateColormapStrip()
{
  if ( colormap_strip_ctl ) {

    colormap_strip_ctl->setPixmap( colormap_pixmap_.isNull() ?
            colormap_pixmap_ :
            colormap_pixmap_.scaled(mtfSlider_ctl->width(), colormap_pixmap_.height()));

    colormap_strip_ctl->setMinimumSize(64, 16);
  }
}

void QMtfControl::resizeEvent(QResizeEvent *e)
{
  Base::resizeEvent(e);
  updateColormapStrip();
}

void QMtfControl::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateHistogramLevels();
}

void QMtfControl::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
}


void QMtfControl::updateControls()
{
  if( !displaySettings_ ) {
    setEnabled(false);
  }
  else {

    double minval, maxval;

    const bool wasInUpdatingControls =
        updatingControls();

    setUpdatingControls(true);

    displayType_ctl->setCurrentIndex(displayType_ctl->findData(
        displaySettings_->displayType()));

    const c_pixinsight_mtf & mtf =
        displaySettings_->mtf();

    mtf.get_input_range(&minval, &maxval);
    inputDataRange_ctl->setText(QString("%1;%2").arg(minval).arg(maxval));

    mtfSlider_ctl->setup(mtf.shadows(), mtf.highlights(), mtf.midtones());

    spins[SPIN_SHADOWS]->setValue(mtf.shadows());
    spins[SPIN_HIGHLIGHTS]->setValue(mtf.highlights());
    spins[SPIN_MIDTONES]->setValue(mtf.midtones());
    logScaleSelectionAction_->setChecked(levelsView_ctl->logScale());

    const bool autoMtfChecked =
        autoMtf_ctl->isChecked();

    spins[SPIN_SHADOWS]->setEnabled(!autoMtfChecked);
    spins[SPIN_HIGHLIGHTS]->setEnabled(!autoMtfChecked);
    spins[SPIN_MIDTONES]->setEnabled(!autoMtfChecked);
    mtfSlider_ctl->setEnabled(!autoMtfChecked);

    switch (selectedAutoMtfAction_) {
    case AutoMtfAction_AutoMtf:
      autoMtf_ctl->setIcon(auto_mtf_icon);
      break;
    case AutoMtfAction_AutoClip:
      default:
      autoMtf_ctl->setIcon(auto_clip_icon);
      break;
    }

    updateHistogramLevels();
    setUpdatingControls(wasInUpdatingControls);
    updateColormapPixmap();
    setEnabled(true);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////

QMtfControlDialogBox::QMtfControlDialogBox(QWidget * parent) :
    Base(parent)
{
  intit_mtfcontrol_resources();

  setWindowIcon(getIcon(ICON_histogram));
  setWindowTitle("Select visual data and adjust display levels ...");

  vbox_ = new QVBoxLayout(this);
  mtfControl_ = new QMtfControl(this);
  vbox_->addWidget(mtfControl_);
}

QMtfControl * QMtfControlDialogBox::mtfControl() const
{
  return mtfControl_;
}

void QMtfControlDialogBox::setMtfDisplaySettings(QMtfDisplay * display)
{
  mtfControl_->setDisplaySettings(display);
}

QMtfDisplay * QMtfControlDialogBox::mtfDisplaySettings() const
{
  return mtfControl_->displaySettings();
}

void QMtfControlDialogBox::showEvent(QShowEvent *event)
{
  if( !lastWidnowSize_.isEmpty() ) {
    Base::move(lastWidnowPos_);
    Base::resize(lastWidnowSize_);
  }

  Base::showEvent(event);
  emit visibilityChanged(isVisible());
}

void QMtfControlDialogBox::hideEvent(QHideEvent *event)
{
  lastWidnowSize_ = this->size();
  lastWidnowPos_ = this->pos();

  Base::hideEvent(event);
  emit visibilityChanged(isVisible());
}
