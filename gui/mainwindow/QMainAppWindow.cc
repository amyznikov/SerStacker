/*
 * QMainAppWindow.cc
 *
 *  Created on: Apr 14, 2023
 *      Author: amyznikov
 */

#include "QMainAppWindow.h"
#include <core/debug.h>


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


  menuFile = mbar->addMenu("&File");
  menuEdit = mbar->addMenu("&Edit");
  menuView = mbar->addMenu("&View");

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
  logWidgetDock =
      addDock<QLogWidgetDock>(this,
          Qt::BottomDockWidgetArea,
          "logwidgetDock_",
          "Debug log",
          logWidget_ctl = new QLogWidget(this),
          menuView);

  //logWidgetDock_->titleBar()->setWindowIcon(getIcon(ICON_log));

  showLogWidgetAction = logWidgetDock->toggleViewAction();
  //showLogWidgetAction_->setIcon(getIcon(ICON_log));

  connect(logWidgetDock, &QDockWidget::visibilityChanged,
      this, &ThisClass::onLogWidgetVisibilityChanged);

  logWidgetDock->hide();
}

void QMainAppWindow::onLogWidgetVisibilityChanged(bool visible)
{
  if( showLogWidgetAction ) {
    showLogWidgetAction->setChecked(visible);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupImageProcessingControls()
{
  imageProcessorDock =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "imageProcessorDock_",
          "Image Processing",
          imageProcessor_ctl = new QImageProcessorSelector(this),
          menuView);

  imageProcessorDock->titleBar()->setWindowIcon(getIcon(ICON_display));
  showImageProcessorAction = imageProcessorDock->toggleViewAction();
  showImageProcessorAction->setIcon(getIcon(ICON_display));
  showImageProcessorAction->setToolTip("Show / Hide display image processing controls");

  connect(showImageProcessorAction, &QAction::triggered,
      this, &ThisClass::onShowImageProcessorControlActionTriggered);

  connect(imageProcessorDock, &QDockWidget::visibilityChanged,
      this, &ThisClass::onImageProcessorControlVisibilityChanged);

  connect(imageProcessor_ctl, &QImageProcessorSelector::parameterChanged,
      this, &ThisClass::onImageProcessorParameterChanged);

  imageProcessorDock->hide();

}

void QMainAppWindow::onShowImageProcessorControlActionTriggered(bool /*checked*/)
{
}

void QMainAppWindow::onImageProcessorControlVisibilityChanged(bool /*visible*/)
{
}

void QMainAppWindow::onImageProcessorParameterChanged()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupDataProcessingControls()
{

  dataframeProcessorDock =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "dataframeProcessorDock_",
          "Data Processing",
          dataframeProcessor_ctl = new QDataFrameProcessorSelector(this),
          menuView);

  dataframeProcessorDock->titleBar()->setWindowIcon(getIcon(ICON_display));
  showDataframeProcessorAction = dataframeProcessorDock->toggleViewAction();
  showDataframeProcessorAction->setIcon(getIcon(ICON_display));
  showDataframeProcessorAction->setToolTip("Show / Hide data frame processing controls");

  connect(showDataframeProcessorAction, &QAction::triggered,
      this, &ThisClass::onShowDataframeProcessorControlActionTriggered);

  connect(dataframeProcessorDock, &QDockWidget::visibilityChanged,
      this, &ThisClass::onDataframeProcessorControlVisibilityChanged);

  connect(dataframeProcessor_ctl, &QDataFrameProcessorSelector::parameterChanged,
      this, &ThisClass::onDataframeProcessorParameterChanged);

  dataframeProcessorDock->hide();
}

void QMainAppWindow::onShowDataframeProcessorControlActionTriggered(bool checked)
{

}

void QMainAppWindow::onDataframeProcessorControlVisibilityChanged(bool visible)
{

}

void QMainAppWindow::onDataframeProcessorParameterChanged()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void QMainAppWindow::setupMeasures()
{
  /////////

  measuresGraphDock =
      addDock<QMeasureGraphDock>(this,
          Qt::RightDockWidgetArea,
          "measureGraphDock_",
          "Measure Graph",
          measuresGraph = new QMeasureGraph(this));

  measuresGraphDock->titleBar()->setWindowIcon(getIcon(ICON_measure_chart));

  /////////

  showMeasuresSettingsAction =
      createCheckableAction(getIcon(ICON_measure_options),
          "Measure Options",
          "Show / Hide measure options",
          is_visible(measureSettingsDisplay),
          this,
          &ThisClass::onShowMeasureSettingsActionTriggered);

  showMeasuresDisplayAction =
      createCheckableAction(getIcon(ICON_measures_table),
          "Measures",
          "Show / Hide measures display",
          is_visible(measureDisplay),
          this,
          &ThisClass::onShowMeasureDisplayActionTriggered);

  showMeasuresGraphAction = measuresGraphDock->toggleViewAction();
  showMeasuresGraphAction->setIcon(getIcon(ICON_measure_chart));

  /////////

  measuresMenu.addAction(showMeasuresSettingsAction);
  measuresMenu.addAction(showMeasuresDisplayAction);
  measuresMenu.addAction(showMeasuresGraphAction);
  measuresMenu.addSeparator();

  if( menuView ) {
    measuresMenuAction = menuView->addMenu(&measuresMenu);
    measuresMenuAction->setText("Measure");
    measuresMenuAction->setIcon(getIcon(ICON_measures));
  }


  /////////

  connect(measuresGraphDock, &QDockWidget::visibilityChanged,
      this, &ThisClass::onMeasuresGraphVisibilityChanged);

  measuresGraphDock->hide();
}


void QMainAppWindow::onShowMeasureSettingsActionTriggered(bool checked)
{
  if ( !checked ) {
    if ( measureSettingsDisplay ) {
      measureSettingsDisplay->hide();
    }
  }
  else {
    if ( !measureSettingsDisplay ) {

      measureSettingsDisplay = new QMeasureSettingsDialogBox(this);
      // measureSettingsDisplay_->resize(QApplication::primaryScreen()->geometry().size() / 2);

      connect(measureSettingsDisplay, &QMeasureSettingsDialogBox::visibilityChanged,
          this, &ThisClass::onMeasureSettingsVisibilityChanged);
    }

    measureSettingsDisplay->show();
    measureSettingsDisplay->raise();
    measureSettingsDisplay->setFocus();
  }
}

void QMainAppWindow::onShowMeasureDisplayActionTriggered(bool checked)
{
  if ( !checked ) {
    if ( measureDisplay ) {
      measureDisplay->hide();
    }
  }
  else {
    if ( !measureDisplay ) {

      measureDisplay = new QMeasureDisplayDialogBox(this);
      measureDisplay->resize(QApplication::primaryScreen()->geometry().size() / 2);

      connect(measureDisplay, &QMeasureDisplayDialogBox::visibilityChanged,
          this, &ThisClass::onMeasuresDisplayVisibilityChanged);

      connect(measureDisplay, &QMeasureDisplayDialogBox::measureRightNowRequested,
          this, &ThisClass::onMeasureRightNowRequested);
    }

    measureDisplay->show();
    measureDisplay->raise();
    measureDisplay->setFocus();
  }
}

void QMainAppWindow::onMeasureSettingsVisibilityChanged(bool visible)
{
  if ( measureSettingsDisplay ) {
    showMeasuresSettingsAction->setChecked(visible);
  }
}

void QMainAppWindow::onMeasuresDisplayVisibilityChanged(bool visible)
{
  if ( showMeasuresDisplayAction ) {
    showMeasuresDisplayAction->setChecked(visible);
  }
}

void QMainAppWindow::onMeasuresGraphVisibilityChanged(bool visible)
{
  if( measuresGraphDock ) {
    measuresGraphDock->toggleViewAction()->setChecked(visible);
  }
}

void QMainAppWindow::onMeasureRightNowRequested()
{
  CF_DEBUG("QMainAppWindow::MeasureRequested");
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QToolButton* QMainAppWindow::createMtfControlButton()
{
  QToolButton * tb =
      createCheckableToolButtonWithContextMenu(getIcon(ICON_histogram),
          "Display Options...",
          "Show / Hide Display Options",
          is_visible(mtfControl),
          [this](QToolButton * tb) {
            if ( showMtfControlAction ) {
              showMtfControlAction->trigger();
            }
          },
          [this](QToolButton * t, const QPoint & pos) {
            onShowMtfControlActionContextMenuRequested(t, pos);
          });


  if( showMtfControlAction ) {
    connect(showMtfControlAction, &QAction::changed,
        [this, tb]() {
          if ( showMtfControlAction ) {
            tb->blockSignals(true);
            tb->setChecked(showMtfControlAction->isChecked());
            tb->blockSignals(false);
          }
        });
  }

  return tb;
}

void QMainAppWindow::setupMtfControls()
{
  if ( !showMtfControlAction ) {
    showMtfControlAction =
        createCheckableAction(getIcon(ICON_histogram),
            "Display Options...",
            "Show / Hide Display Options",
            is_visible(mtfControl),
            this,
            &ThisClass::onShowMtfControlActionTriggered);


    if( menuView ) {
      menuView->addAction(showMtfControlAction);
    }
  }
}

void QMainAppWindow::onShowMtfControlActionTriggered(bool checked)
{
  if( !checked ) {
    if( mtfControl ) {
      mtfControl->hide();
    }
  }
  else {
    if( !mtfControl ) {

      mtfControl = new QMtfControlDialogBox(this);

      connect(mtfControl, &QMtfControlDialogBox::visibilityChanged,
          this, &ThisClass::onMtfControlVisibilityChanged);
    }

    mtfControl->show();
    mtfControl->raise();
    mtfControl->setFocus();
  }
}

void QMainAppWindow::onShowMtfControlActionContextMenuRequested(QToolButton * tb, const QPoint & pos)
{
  if( !is_visible(mtfControl) ) {

    IMtfDisplay * mtfDisplay =
        getCurrentMtfDisplay();

    if( mtfDisplay ) {

      const QStringList availableDisplayChannels =
          mtfDisplay->displayChannels();

      if( availableDisplayChannels.size() > 1 ) {

        QMenu menu;

        const QString & currentDisplayChannel =
            mtfDisplay->displayChannel();

        for( const QString & s : availableDisplayChannels ) {

          menu.addAction(createCheckableAction(QIcon(), s, "",
              s == currentDisplayChannel,
              [s, mtfDisplay](bool checked) {
                if ( checked ) {
                  mtfDisplay->setDisplayChannel(s);
                }
              }));
        }

        if( !menu.isEmpty() ) {
          menu.exec(tb->mapToGlobal(QPoint(pos.x() - 2, pos.y() - 2)));
        }

      }
    }
  }
}

void QMainAppWindow::onMtfControlVisibilityChanged(bool visible)
{
  if( showMtfControlAction ) {
    showMtfControlAction->setChecked(visible);
  }
}

IMtfDisplay * QMainAppWindow::getCurrentMtfDisplay()
{
  return nullptr;
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


    measuresMenu.addAction(showProfileGraphAction_);

    if( menuView && !measuresMenuAction ) {
      measuresMenuAction = menuView->addMenu(&measuresMenu);
      measuresMenuAction->setText("Measure");
      measuresMenuAction->setIcon(getIcon(ICON_measures));
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
      profileGraph_ctl = plotProfileDialogBox_->profileGraph();

      connect(plotProfileDialogBox_, &QProfileGraphDialogBox::visibilityChanged,
          this, &ThisClass::onPlotProfileDialogBoxVisibilityChanged);

//      connect(profileGraph_ctl_, &QProfileGraph::skipZeroPixlelsChanged,
//          [this]() {
//            updateProfileGraph();
//          });

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

QWidget* QMainAppWindow::createStretch()
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return stretch;
}

QWidget* QMainAppWindow::addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}



