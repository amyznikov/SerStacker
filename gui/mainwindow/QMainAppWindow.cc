/*
 * QMainAppWindow.cc
 *
 *  Created on: Apr 14, 2023
 *      Author: amyznikov
 */

#include "QMainAppWindow.h"

//#define ICON_log                  ":/qlog/icons/log.png"
#define ICON_histogram            ":/qmtf/icons/histogram_small.png"
#define ICON_display              ":/qimproc/icons/display.png"
#define ICON_process              ":/qimproc/icons/process.png"
#define ICON_measures             ":/qmeasure/icons/measure.png"
#define ICON_measures_table       ":/qmeasure/icons/table.png"
#define ICON_measure_clear        ":/qmeasure/icons/clear.png"
#define ICON_measure_chart        ":/qmeasure/icons/chart.png"
#define ICON_plot                 ":/qmeasure/icons/plot.png"
#define ICON_measure_options      ":/qmeasure/icons/options.png"
//#define ICON_measure_roi          ":/qmeasure/icons/roi.png"
//#define ICON_measure_menu         ":/qmeasure/icons/menu.png"


QMainAppWindow::QMainAppWindow(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qlog_resources);
  Q_INIT_RESOURCE(qmtf_resources);
  Q_INIT_RESOURCE(qmeasure_resources);
  Q_INIT_RESOURCE(qimproc_resources);

  QApplication::instance()->installEventFilter(this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool QMainAppWindow::eventFilter(QObject * watched, QEvent * event)
{
  if( event->type() == QEvent::Wheel ) {

    if( const auto *combo = dynamic_cast<const QComboBox*>(watched) ) {
      if( !combo->hasFocus() ) {
        return true;
      }
    }
    //  spin_ctl->setFocusPolicy(Qt::FocusPolicy::StrongFocus) not works
    else if( const auto *spin = dynamic_cast<const QSpinBox*>(watched) ) {
      if( !spin->hasFocus() ) {
        return true;
      }
    }
    else if( const auto *slider = dynamic_cast<const QSlider*>(watched) ) {
      if( !slider->hasFocus() ) {
        return true;
      }
    }
  }

  return Base::eventFilter(watched, event);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::saveState()
{
  QSettings settings;
  onSaveState(settings);
}

void QMainAppWindow::onSaveState(QSettings & settings)
{
  settings.setValue("MainWindow/State", Base::saveState());
  settings.setValue("MainWindow/Geometry", Base::saveGeometry());
}

void QMainAppWindow::restoreState()
{
  QSettings settings;
  onRestoreState(settings);
}

void QMainAppWindow::onRestoreState(QSettings & settings)
{
  Base::restoreGeometry(settings.value("MainWindow/Geometry").toByteArray());
  Base::restoreState(settings.value("MainWindow/State").toByteArray());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupMainMenu()
{
  QMenuBar * mbar =
      menuBar();

  mbar->setNativeMenuBar(false);


  menuFile_ = mbar->addMenu("&File");
  menuEdit_ = mbar->addMenu("&Edit");
  menuView_ = mbar->addMenu("&View");

}


void QMainAppWindow::setupMainToolbar()
{
}

void QMainAppWindow::setupStatusbar()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupLogWidget()
{
  logWidgetDock_ =
      addDock<QLogWidgetDock>(this,
          Qt::BottomDockWidgetArea,
          "logwidgetDock_",
          "Debug log",
          logWidget_ctl = new QLogWidget(this),
          menuView_);

  //logWidgetDock_->titleBar()->setWindowIcon(getIcon(ICON_log));

  showLogWidgetAction_ = logWidgetDock_->toggleViewAction();
  //showLogWidgetAction_->setIcon(getIcon(ICON_log));

  connect(logWidgetDock_, &QDockWidget::visibilityChanged,
      this, &ThisClass::onLogWidgetVisibilityChanged);

  logWidgetDock_->hide();
}

void QMainAppWindow::onLogWidgetVisibilityChanged(bool visible)
{
  if( showLogWidgetAction_ ) {
    showLogWidgetAction_->setChecked(visible);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupImageProcessingControls()
{
  imageProcessorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "imageProcessorDock_",
          "Image Processing",
          imageProcessor_ctl = new QImageProcessorSelector(this),
          menuView_);

  imageProcessorDock_->titleBar()->setWindowIcon(getIcon(ICON_display));
  showImageProcessorAction_ = imageProcessorDock_->toggleViewAction();
  showImageProcessorAction_->setIcon(getIcon(ICON_display));
  showImageProcessorAction_->setToolTip("Show / Hide display image processing controls");

  connect(showImageProcessorAction_, &QAction::triggered,
      this, &ThisClass::onShowImageProcessorControlActionTriggered);

  connect(imageProcessorDock_, &QDockWidget::visibilityChanged,
      this, &ThisClass::onImageProcessorControlVisibilityChanged);

  connect(imageProcessor_ctl, &QImageProcessorSelector::parameterChanged,
      this, &ThisClass::onImageProcessorParameterChanged);

  imageProcessorDock_->hide();

}

void QMainAppWindow::onShowImageProcessorControlActionTriggered(bool /*checked*/)
{
}

void QMainAppWindow::onImageProcessorControlVisibilityChanged(bool /*visible*/)
{
}

void QMainAppWindow::onImageProcessorParameterChanged()
{
  //centralDisplay_->setFrameProcessor(frameProcessor_ctl->current_processor());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupMeasures()
{
  /////////

  measuresGraphDock_ =
      addDock<QMeasureGraphDock>(this,
          Qt::RightDockWidgetArea,
          "measureGraphDock_",
          "Measure Graph",
          measuresGraph_ = new QMeasureGraph(this));

  measuresGraphDock_->titleBar()->setWindowIcon(getIcon(ICON_measure_chart));

  /////////

  showMeasuresSettingsAction_ =
      createCheckableAction(getIcon(ICON_measure_options),
          "Measure Options",
          "Show / Hide measure options",
          is_visible(measureSettingsDisplay_),
          this,
          &ThisClass::onShowMeasureSettingsActionTriggered);

  showMeasuresDisplayAction_ =
      createCheckableAction(getIcon(ICON_measures_table),
          "Measures",
          "Show / Hide measures display",
          is_visible(measureDisplay_),
          this,
          &ThisClass::onShowMeasureDisplayActionTriggered);

  showMeasuresGraphAction_ = measuresGraphDock_->toggleViewAction();
  showMeasuresGraphAction_->setIcon(getIcon(ICON_measure_chart));

  /////////

  measuresMenu_.addAction(showMeasuresSettingsAction_);
  measuresMenu_.addAction(showMeasuresDisplayAction_);
  measuresMenu_.addAction(showMeasuresGraphAction_);
  measuresMenu_.addSeparator();

  if( menuView_ ) {
    measuresMenuAction_ = menuView_->addMenu(&measuresMenu_);
    measuresMenuAction_->setText("Measure");
    measuresMenuAction_->setIcon(getIcon(ICON_measures));
  }


  /////////

  connect(measuresGraphDock_, &QDockWidget::visibilityChanged,
      this, &ThisClass::onMeasuresGraphVisibilityChanged);

  measuresGraphDock_->hide();
}


void QMainAppWindow::onShowMeasureSettingsActionTriggered(bool checked)
{
  if ( !checked ) {
    if ( measureSettingsDisplay_ ) {
      measureSettingsDisplay_->hide();
    }
  }
  else {
    if ( !measureSettingsDisplay_ ) {

      measureSettingsDisplay_ = new QMeasureSettingsDialogBox(this);
      // measureSettingsDisplay_->resize(QApplication::primaryScreen()->geometry().size() / 2);

      connect(measureSettingsDisplay_, &QMeasureSettingsDialogBox::visibilityChanged,
          this, &ThisClass::onMeasureSettingsVisibilityChanged);
    }

    measureSettingsDisplay_->show();
    measureSettingsDisplay_->raise();
    measureSettingsDisplay_->setFocus();
  }
}

void QMainAppWindow::onShowMeasureDisplayActionTriggered(bool checked)
{
  if ( !checked ) {
    if ( measureDisplay_ ) {
      measureDisplay_->hide();
    }
  }
  else {
    if ( !measureDisplay_ ) {

      measureDisplay_ = new QMeasureDisplayDialogBox(this);
      measureDisplay_->resize(QApplication::primaryScreen()->geometry().size() / 2);

      connect(measureDisplay_, &QMeasureDisplayDialogBox::visibilityChanged,
          this, &ThisClass::onMeasuresDisplayVisibilityChanged);

      CF_DEBUG("connect onMeasureRightNowRequested");

      connect(measureDisplay_, &QMeasureDisplayDialogBox::measureRightNowRequested,
          this, &ThisClass::onMeasureRightNowRequested);
    }

    measureDisplay_->show();
    measureDisplay_->raise();
    measureDisplay_->setFocus();
  }
}

void QMainAppWindow::onMeasureSettingsVisibilityChanged(bool visible)
{
  if ( measureSettingsDisplay_ ) {
    showMeasuresSettingsAction_->setChecked(visible);
  }
}

void QMainAppWindow::onMeasuresDisplayVisibilityChanged(bool visible)
{
  if ( showMeasuresDisplayAction_ ) {
    showMeasuresDisplayAction_->setChecked(visible);
  }
}

void QMainAppWindow::onMeasuresGraphVisibilityChanged(bool visible)
{
  if( measuresGraphDock_ ) {
    measuresGraphDock_->toggleViewAction()->setChecked(visible);
  }
}

void QMainAppWindow::onMeasureRightNowRequested()
{
  CF_DEBUG("QMainAppWindow::MeasureRequested");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupMtfControls()
{
  if ( !showMtfControlAction_ ) {
    showMtfControlAction_ =
        createCheckableAction(getIcon(ICON_histogram),
            "Display Options...",
            "Show / Hide Display Options",
            is_visible(mtfControl_),
            this,
            &ThisClass::onShowMtfControlActionTriggered);


    if( menuView_ ) {
      menuView_->addAction(showMtfControlAction_);
    }
  }
}

void QMainAppWindow::onShowMtfControlActionTriggered(bool checked)
{
  if( !checked ) {
    if( mtfControl_ ) {
      mtfControl_->hide();
    }
  }
  else {
    if( !mtfControl_ ) {

      mtfControl_ = new QMtfControlDialogBox(this);

      connect(mtfControl_, &QMtfControlDialogBox::visibilityChanged,
          this, &ThisClass::onMtfControlVisibilityChanged);
    }

    mtfControl_->show();
    mtfControl_->raise();
    mtfControl_->setFocus();
  }
}

void QMainAppWindow::onMtfControlVisibilityChanged(bool visible)
{
  if( showMtfControlAction_ ) {
    showMtfControlAction_->setChecked(visible);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void QMainAppWindow::setupProfileGraph()
{
  if ( !showProfileGraphAction_ ) {

    showProfileGraphAction_ =
        createCheckableAction(getIcon(ICON_plot),
            "Plot Profile ...",
            "Show / Hide plot profile widget",
            is_visible(plotProfileDialogBox_),
            this,
            &ThisClass::onShowProfileGraphActionTriggered);


    measuresMenu_.addAction(showProfileGraphAction_);

    if( menuView_ && !measuresMenuAction_ ) {
      measuresMenuAction_ = menuView_->addMenu(&measuresMenu_);
      measuresMenuAction_->setText("Measure");
      measuresMenuAction_->setIcon(getIcon(ICON_measures));
    }

  }
}

void QMainAppWindow::onShowProfileGraphActionTriggered(bool checked)
{
  if( !checked ) {
    if( plotProfileDialogBox_ ) {
      plotProfileDialogBox_->hide();
    }
  }
  else {
    if( !plotProfileDialogBox_ ) {

      plotProfileDialogBox_ = new QProfileGraphDialogBox(this);
      profileGraph_ctl_ = plotProfileDialogBox_->profileGraph();

      connect(plotProfileDialogBox_, &QProfileGraphDialogBox::visibilityChanged,
          this, &ThisClass::onPlotProfileDialogBoxVisibilityChanged);

      connect(profileGraph_ctl_, &QProfileGraph::skipZeroPixlelsChanged,
          [this]() {
            updateProfileGraph();
          });

    }

    plotProfileDialogBox_->show();
    plotProfileDialogBox_->raise();
    plotProfileDialogBox_->setFocus();
  }
}

void QMainAppWindow::onPlotProfileDialogBoxVisibilityChanged(bool visible)
{
  if( showProfileGraphAction_ ) {
    showProfileGraphAction_->setChecked(visible);
  }

  if ( visible ) {
    updateProfileGraph();
  }
}

void QMainAppWindow::updateProfileGraph(QGraphicsItem * /*lineItem*/)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QToolButton* QMainAppWindow::createToolButtonWithPopupMenu(QAction * defaultAction, QMenu * menu)
{
  QToolButton *tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(defaultAction->icon());
  tb->setText(defaultAction->text());
  tb->setToolTip(defaultAction->toolTip());
  tb->setCheckable(defaultAction->isCheckable());
  tb->setPopupMode(QToolButton::ToolButtonPopupMode::MenuButtonPopup);
  tb->setDefaultAction(defaultAction);
  tb->setMenu(menu);
  return tb;
}

QToolButton* QMainAppWindow::createToolButtonWithMenu(const QIcon & icon, const QString & text, const QString & tooltip, QMenu * menu)
{
  QToolButton *tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(icon);
  tb->setText(text);
  tb->setToolTip(tooltip);

  if ( menu ) {
    QObject::connect(tb, &QToolButton::clicked,
        [tb, menu]() {
          menu->exec(tb->mapToGlobal(QPoint( tb->width()/2, tb->height() - 2 )));
        });
  }

  return tb;
}

QWidget* QMainAppWindow::addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}

QWidget* QMainAppWindow::createStretch()
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return stretch;
}


