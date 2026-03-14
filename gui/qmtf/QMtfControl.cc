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
#include <gui/widgets/qsprintf.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/proc/histogram-tools.h>
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


static void setValue(QDoubleSpinBox * ctl, double v)
{
  QSignalBlocker block(ctl);
  ctl->setValue(v);
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
  if ( colormap_icon.isNull() ) {
    colormap_icon = getIcon(ICON_colormap);
  }

}


/////////////////////////////////////////////////////////////////////////////////////////

QMtfControl::QMtfControl(QWidget * parent) :
    Base(parent)
{
  intit_mtfcontrol_resources();

  _vbox = new QVBoxLayout(this);

  topToolbar_ctl = new QToolBar(this);
  topToolbar_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  topToolbar_ctl->setIconSize(QSize(16, 16));


  displayChannel_ctl = new QComboBox(this);
  displayChannel_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  displayChannel_ctl->setToolTip("Select data to visualize");
  topToolbar_ctl->addWidget(displayChannel_ctl);

  connect(displayChannel_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onDisplayChannelCurrentItemChanged);

  inputDataRange_ctl = new QNumericBox(this);
  inputDataRange_ctl->setToolTip("Set input data range (min/max clips)");
  topToolbar_ctl->addWidget(inputDataRange_ctl);
  connect(inputDataRange_ctl, &QNumericBox::textChanged,
      this, &ThisClass::onInputDataRangeChanged);


  _resetMtfAction = topToolbar_ctl->addAction(getIcon(ICON_reset), "Reset");
  _resetMtfAction->setToolTip("Reset MTF clipping and recompute input data range");
  connect(_resetMtfAction, &QAction::triggered,
      this, &ThisClass::onResetMtfClicked);

  _autoMtffAction = topToolbar_ctl->addAction(getIcon(ICON_histogram_automtf), "Auto MTF");
  _autoMtffAction->setToolTip("Auto MTF adjustment");
  connect( _autoMtffAction, &QAction::triggered,
      this, &ThisClass::onAutoMtfCtrlClicked);


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
  mtfSliderBand_ctl = new QMtfSliderBand(this);
  colormap_strip_ctl = new QLabel(this);
  //  colormap_strip_ctl->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

  // configure SpinBox toolbars
  bottomToolbar_ctl = new QToolBar(this);
  bottomToolbar_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  bottomToolbar_ctl->setIconSize(QSize(16, 16));

  static const auto createSpinBox =
      [](QWidget * parent, double minv, double maxv, double v, int decimals) -> QDoubleSpinBox* {
        QDoubleSpinBox * ctl = new QDoubleSpinBox(parent);
        ctl->setKeyboardTracking(false);
        ctl->setRange(minv, maxv);
        ctl->setDecimals(decimals);
        ctl->setSingleStep(std::pow(1.0, -decimals));
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
        ctl->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
#endif
        ctl->setValue(v);
        return ctl;
      };

  bottomToolbar_ctl->addWidget(lclip_ctl = createSpinBox(this, 0, 1, 0, 3));
  bottomToolbar_ctl->addWidget(shadows_ctl = createSpinBox(this, -2, 2, 0, 2));
  addStretch(bottomToolbar_ctl);
  bottomToolbar_ctl->addWidget(midtones_ctl = createSpinBox(this, 0, 1, 0.5, 3));
  addStretch(bottomToolbar_ctl);
  bottomToolbar_ctl->addWidget(highlights_ctl = createSpinBox(this, -2, 2, 0, 2));
  bottomToolbar_ctl->addWidget(hclip_ctl = createSpinBox(this, 0, 1, 1, 3));

  // statusBar
  statusBar_ctl = new QStatusBar(this);

  // Combine layouts
  _vbox->addWidget(topToolbar_ctl, 1);
  _vbox->addWidget(levelsView_ctl, 1000);
  _vbox->addWidget(mtfSliderBand_ctl, 1);
  _vbox->addWidget(colormap_strip_ctl, 1);
  _vbox->addWidget(bottomToolbar_ctl, 1);
  _vbox->addWidget(statusBar_ctl, 1);


  // Configure event handles

  connect(logScaleSelectionAction_, &QAction::triggered,
      levelsView_ctl, &QHistogramView::setLogScale);

  connect(mtfSliderBand_ctl, &QMtfSliderBand::positonChanged,
      [this](int slider, double value) {
        if ( _displaySettings && !updatingControls() ) {

          c_update_controls_lock lock(this);

          const double shadows = shadows_ctl->value();
          const double highlights = highlights_ctl->value();

          if ( slider == QMtfSliderBand::SLIDER_LCLIP ) {
            const double v = mtfSliderBand_ctl->lclip();
            setValue(lclip_ctl, v);
            _displaySettings->setlclip(v);
          }
          else if ( slider == QMtfSliderBand::SLIDER_HCLIP ) {
            const double v = mtfSliderBand_ctl->hclip();
            setValue(hclip_ctl, v);
            _displaySettings->sethclip(v);
          }
          else if ( slider == QMtfSliderBand::SLIDER_MIDTONES ) {
            const double v = mtfSliderBand_ctl->midtones();
            setValue(midtones_ctl, v);
            _displaySettings->setMidtones(v);
          }
        }
      });

  connect(lclip_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( _displaySettings && !updatingControls() ) {
          c_update_controls_lock lock(this);
          mtfSliderBand_ctl->setlclip(v);
          _displaySettings->setlclip(v);
        }
      });

  connect(hclip_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( _displaySettings && !updatingControls() ) {
          c_update_controls_lock lock(this);
          mtfSliderBand_ctl->sethclip(v);
          _displaySettings->sethclip(v);
        }
      });

  connect(midtones_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( _displaySettings && !updatingControls() ) {
          c_update_controls_lock lock(this);
          mtfSliderBand_ctl->setMidtones(v);
          _displaySettings->setMidtones(v);
        }
      });

  connect(shadows_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( _displaySettings && !updatingControls() ) {
          c_update_controls_lock lock(this);
          _displaySettings->setShadows(v);
        }
      });

  connect(highlights_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( _displaySettings && !updatingControls() ) {
          c_update_controls_lock lock(this);
          _displaySettings->setHighlights(v);
        }
      });

  displayChannel_ctl->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);
  connect(displayChannel_ctl, &QWidget::customContextMenuRequested,
      this, &ThisClass::onDisplayChannelCustomContextMenuRequested);


  //displayChannel_ctl->customContextMenuRequested(pos)

  updateControls();
}

void QMtfControl::setDisplaySettings(IMtfDisplay * displaySettings)
{
  if( displaySettings != this->_displaySettings ) {

    c_update_controls_lock lock(this);

    if( QObject * qobj = dynamic_cast<QObject*>(this->_displaySettings) ) {
      qobj->disconnect(this);
    }

    _displaySettings = displaySettings;
    displayChannel_ctl->clear();

    if( QObject * qobj = dynamic_cast<QObject*>(_displaySettings) ) {

      displayChannel_ctl->addItems(_displaySettings->displayChannels());

      connect(qobj, SIGNAL(displayImageChanged()),
          this, SLOT(onDisplayImageChanged()));

      connect(qobj, SIGNAL(displayChannelsChanged()),
          this, SLOT(onDisplayChannelsChanged()));

    }

    updateControls();
    updateHistogramLevels();
  }
}

IMtfDisplay * QMtfControl::displaySettings() const
{
  return _displaySettings;
}

void QMtfControl::setHistoramViewSizeHint(const QSize & s)
{
  levelsView_ctl->setSizeHint(s);
}

QSize QMtfControl::historamViewSizeHint() const
{
  return levelsView_ctl->sizeHint();
}

void QMtfControl::onChartTypeSelectorClicked()
{
  static QMenu chartTypesMenu;
  static QAction *setLineChartAction = nullptr;
  static QAction *setBarChartAction = nullptr;

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
  if( _displaySettings ) {

    c_mtf_options opts;
    double irange[2] = {-1, -1};

    _displaySettings->getMtfInputRange(&irange[0], &irange[1]);

    if( irange[0] < irange[1] ) {
      irange[0] = irange[1] = -1;
    }
    else {
      QWaitCursor wait(this);
      _displaySettings->getInputDataRange(&irange[0], &irange[1]);
    }

    _displaySettings->setMtf(irange[0], irange[1], &opts);
    _displaySettings->saveParameters();

    updateControls();
  }
}

void QMtfControl::onAutoMtfCtrlClicked()
{
  if ( _displaySettings ) {

    double hmin = -1, hmax = -1;
    c_mtf_options opts;
    cv::Mat1d H;

    _displaySettings->getInputHistogramm(H, &hmin, &hmax);
    if( H.empty() || !(hmax > hmin) ) {
      CF_ERROR("_displaySettings->getInputHistogramm() fails");
    }
    else {

      if( H.cols > 1 ) {
        cv::reduce(H, H, 1, cv::REDUCE_AVG);
      }

      double lc, hc, s, m, h;

      autoMtf(H, hmin, hmax, &lc, &hc, &s, &m, &h);

      CF_DEBUG("H: %dx%d hmin=%g hmax=%g lc=%g, hc=%g, s=%g, m=%g, h=%g", H.rows, H.cols, hmin, hmax, lc, hc, s, m, h);

      const double hrange = (hmax - hmin);
      opts.lclip = (lc - hmin) / hrange;
      opts.hclip = (hc - hmin) / hrange;
      opts.shadows = s;
      opts.highlights = h;
      opts.midtones = m;
      _displaySettings->setMtf(hmin, hmax, &opts);
      _displaySettings->saveParameters();
      updateControls();
    }

    updateControls();
  }
}

void QMtfControl::onColormapCtlClicked()
{
  if( _displaySettings ) {

    QMenu menu;
    QAction * invertColormapAction;

    const COLORMAP current_colormap =
        _displaySettings->colormap();


    menu.addAction(invertColormapAction = new QAction("Invert colormap"));
    invertColormapAction->setCheckable(true);
    invertColormapAction->setChecked(_displaySettings->invertColormap());
    menu.addSeparator();

    for( const c_enum_member *colormap = members_of<COLORMAP>(); !colormap->name.empty(); ++colormap ) {

      QAction * action = new QAction(colormap->name.c_str());
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
        _displaySettings->setInvertColormap(invertColormapAction->isChecked());
      }
      else {
        _displaySettings->setColormap((COLORMAP) action->data().toInt());
      }

      _displaySettings->saveParameters();

      updateColormapPixmap();
    }
  }
}

void QMtfControl::onDisplayChannelCurrentItemChanged()
{
  if( _displaySettings && !updatingControls() ) {

    c_update_controls_lock lock(this);

    double input_range[2] = { -1, -1 };
    c_mtf_options opts;

    getInputDataRangeCtl(input_range);

    _displaySettings->setDisplayChannel(displayChannel_ctl->currentText());
    _displaySettings->saveParameters();

    _displaySettings->getMtf(&opts);
    _displaySettings->getMtfInputRange(&input_range[0], &input_range[1]);

    setValue(lclip_ctl, opts.lclip);
    setValue(hclip_ctl, opts.hclip);
    setValue(midtones_ctl, opts.midtones);
    setValue(shadows_ctl, opts.shadows);
    setValue(highlights_ctl, opts.highlights);

    mtfSliderBand_ctl->setOpts(opts.lclip, opts.hclip, opts.midtones);
    setInputDataRangeCtl(input_range);
  }
}

bool QMtfControl::getInputDataRangeCtl(double range[2]) const
{
  return fromString(inputDataRange_ctl->text(), range, 2) == 2;
}

void QMtfControl::setInputDataRangeCtl(const double range[2])
{
  QSignalBlocker block(inputDataRange_ctl);
  inputDataRange_ctl->setText(QString("%1;%2").arg(range[0]).arg(range[1]));
}


void QMtfControl::onInputDataRangeChanged()
{
  if( _displaySettings && !updatingControls() ) {

    double range[2] = {-1, -1};

    if( getInputDataRangeCtl(range) ) {
      _displaySettings->setMtf(range[0], range[1]);
      _displaySettings->saveParameters();
    }
  }
}



void QMtfControl::onDisplayChannelCustomContextMenuRequested(const QPoint & pos)
{
  if( displayChannel_ctl->count() > 0 ) {

    QMenu menu;

    menu.addAction("Copy",
        [this]() {
          QApplication::clipboard()->setText(displayChannel_ctl->currentText());
        });

    menu.addAction("Copy All",
        [this]() {

          QString text;

          for ( int i = 0, n = displayChannel_ctl->count(); i < n; ++i ) {
            text.append(displayChannel_ctl->itemText(i));
            text.append('\n');
          }

          QApplication::clipboard()->setText(text);
        });

    menu.exec(displayChannel_ctl->mapToGlobal(QPoint(pos.x() - 4, pos.y() - 4)));

  }
}


void QMtfControl::findAutoHistogramClips()
{
  if( _displaySettings ) {
  }
}

void QMtfControl::findAutoMidtonesBalance()
{
  if( _displaySettings ) {
  }
}

void QMtfControl::updateHistogramLevels()
{
  if ( _displaySettings && isVisible() ) {

    cv::Mat1d H;
    std::vector<float> cy;
    double minv = -1, maxv = -1;

    _displaySettings->getOutputHistogramm(H, &minv, &maxv);
    _displaySettings->getMtfCurve(cy, 256);
    levelsView_ctl->setHistogram(H, minv, maxv);
    levelsView_ctl->setMtfCurve(std::move(cy));

    if ( H.cols != 1 ) {
      cv::reduce(H, H, 1, cv::REDUCE_AVG);
    }

    const double mode = computeHistogramMode(H, minv, maxv)[0];

    makeCumulativeHistogram(H, H, true);

    const double med = computeHistogramMedian(H, minv, maxv)[0];

    statusBar_ctl->showMessage(qsprintf("Hx%d mode: %g median: %g", H.rows, mode, med));
  }
}

void QMtfControl::updateColormapPixmap()
{
  if ( _displaySettings ) {

    cv::Mat1b gray_image(16, 256);
    QImage qimage;

    for( int y = 0; y < gray_image.rows; ++y ) {
      for( int x = 0; x < gray_image.cols; ++x ) {
        gray_image[y][x] = x;
      }
    }

    const COLORMAP cmap =
        _displaySettings->colormap();

    if ( cmap == COLORMAP_NONE ) {
      cv2qt(gray_image, &qimage);
    }
    else {
      cv::Mat3b color_image;
      apply_colormap(gray_image, color_image, cmap);
      cv2qt(color_image, &qimage);
    }

    if ( _displaySettings->invertColormap() ) {

      qimage = qimage.mirrored(true, false);
    }

    _colormap_pixmap =
        QPixmap::fromImage(qimage);

  }

  updateColormapStrip();

}

void QMtfControl::updateColormapStrip()
{
  if ( colormap_strip_ctl ) {

    colormap_strip_ctl->setPixmap( _colormap_pixmap.isNull() ?
            _colormap_pixmap :
            _colormap_pixmap.scaled(mtfSliderBand_ctl->width(), _colormap_pixmap.height()));

    colormap_strip_ctl->setMinimumSize(32, 16);
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


void QMtfControl::onDisplayChannelsChanged()
{
  if ( _displaySettings ) {

    c_update_controls_lock lock(this);

    displayChannel_ctl->clear();
    displayChannel_ctl->addItems(_displaySettings->displayChannels());
    displayChannel_ctl->setCurrentIndex(std::max(0, displayChannel_ctl->findText(_displaySettings->displayChannel())));
  }
}

void QMtfControl::onDisplayImageChanged()
{
  updateHistogramLevels();
}


void QMtfControl::onupdatecontrols()
{
  if( !_displaySettings ) {
    setEnabled(false);
  }
  else {

    double irange[2] = {-1, -1};
    c_mtf_options opts;

    const QString & displayChannel = _displaySettings->displayChannel();
    int idx = displayChannel_ctl->findText(displayChannel);

    displayChannel_ctl->setCurrentIndex(idx);

    _displaySettings->getMtfInputRange(&irange[0], &irange[1]);
    _displaySettings->getMtf(&opts);

    setInputDataRangeCtl(irange);

    setValue(lclip_ctl, opts.lclip);
    setValue(shadows_ctl, opts.shadows);
    setValue(midtones_ctl, opts.midtones);
    setValue(highlights_ctl, opts.highlights);
    setValue(hclip_ctl, opts.hclip);
    mtfSliderBand_ctl->setOpts(opts.lclip, opts.hclip, opts.midtones);

    logScaleSelectionAction_->setChecked(levelsView_ctl->logScale());

//    const bool autoMtfChecked = autoMtf_ctl->isChecked();
//    mtfSliderBand_ctl->setEnabled(!autoMtfChecked);

//    switch (selectedAutoMtfAction_) {
//      case AutoMtfAction_AutoMtf:
//        autoMtf_ctl->setIcon(auto_mtf_icon);
//        break;
//      case AutoMtfAction_AutoClip:
//        default:
//        autoMtf_ctl->setIcon(auto_clip_icon);
//        break;
//    }

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

  _vbox = new QVBoxLayout(this);
  _mtfControl = new QMtfControl(this);
  _vbox->addWidget(_mtfControl);
}

QMtfControl * QMtfControlDialogBox::mtfControl() const
{
  return _mtfControl;
}

void QMtfControlDialogBox::setMtfDisplaySettings(IMtfDisplay * display)
{
  _mtfControl->setDisplaySettings(display);
}

IMtfDisplay * QMtfControlDialogBox::mtfDisplaySettings() const
{
  return _mtfControl->displaySettings();
}

void QMtfControlDialogBox::showEvent(QShowEvent *event)
{
  if( !_lastWidnowSize.isEmpty() ) {
    Base::move(_lastWidnowPos);
    Base::resize(_lastWidnowSize);
  }

  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QMtfControlDialogBox::hideEvent(QHideEvent *event)
{
  _lastWidnowSize = this->size();
  _lastWidnowPos = this->pos();

  Base::hideEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}
