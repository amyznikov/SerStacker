/*
 * MainWindow.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/style.h>


namespace serimager {

#define ICON_histogram        ":/qserimager/icons/histogram.png"
#define ICON_display          ":/qserimager/icons/display.png"
#define ICON_roi              ":/qserimager/icons/roi.png"



namespace {

template<class Obj, typename Fn>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip,
    Obj * receiver, Fn fn)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  QObject::connect(action, &QAction::triggered, receiver, fn);

  return action;
}

}

MainWindow::MainWindow(QWidget * parent) :
   Base(parent)
{
  setWindowIcon(getIcon(":/qserimager/icons/app-icon.png"));

  setCentralWidget(centralDisplay_ = new QCameraFrameDisplay(this));


  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  setupMainMenu();
  setupFocusGraph();
  setupIndigoFocuser();
  setupImagerSettings();
  setupFrameProcessorControls();
  setupMainToolbar();

  restoreState();

  QApplication::instance()->installEventFilter(this);

  statusBar()->addWidget(mousepos_ctl = new QLabel("mouse", this));
  mousepos_ctl->setText("");
  //mousepos_ctl->setFrameShape(QFrame::Box);

  connect(centralDisplay_, &QCameraFrameDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mousepos_ctl->setText(centralDisplay_->statusStringForPixel(e->pos()));
        //statusBar()->showMessage(centralDisplay_->statusStringForPixel(e->pos()));
      });

}


MainWindow::~MainWindow()
{
  saveState();
}


bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  if( event->type() == QEvent::Wheel ) {

    if( const auto *combo = dynamic_cast<const QComboBox*>(watched) ) {
      if( !combo->isEditable() ) {
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

void MainWindow::saveState()
{
  QSettings settings;
  settings.setValue("MainWindow/Geometry", Base::saveGeometry());
  settings.setValue("MainWindow/State", Base::saveState());
}

void MainWindow::restoreState()
{
  QSettings settings;
  Base::restoreGeometry(settings.value("MainWindow/Geometry").toByteArray());
  Base::restoreState(settings.value("MainWindow/State").toByteArray());
}

void MainWindow::setupMainMenu()
{
  menuBar()->setNativeMenuBar(false);

  ///////////////////////////////////////////////////////////////////

  menuFile_ =
      menuBar()->addMenu("&File");

  menuFile_->addSeparator();
  menuFile_->addAction("Quit", [this]() {
    close();
  });

  ///////////////////////////////////////////////////////////////////

  menuView_ =
      menuBar()->addMenu("&View");

  /////////////////////////////////////

  menuView_->addAction(showRoiAction_ =
      createCheckableAction(getIcon(ICON_roi),
          "Show ROI",
          "Show / Hide ROI rectangle",
          centralDisplay_,
          &QCameraFrameDisplay::setShowROI));

  showRoiAction_->setChecked(centralDisplay_->showROI());


  /////////////////////////////////////

  menuView_->addAction(showMtfControlAction_ =
      createCheckableAction(getIcon(ICON_histogram),
          "Display Options...",
          "Show / Hide Display Options",
          this,
          &ThisClass::onShowMtfControlActionTriggered));



  /////////////////////////////////////



}

void MainWindow::setupMainToolbar()
{
  menuBar()->setCornerWidget(manToolbar_ =
      new QToolBar(this),
      Qt::TopRightCorner);

  manToolbar_->setContentsMargins(0, 0, 0, 0);
  manToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  manToolbar_->setIconSize(QSize(18, 18));

  ///////////////////////////////////////////////////////////////////
  manToolbar_->addAction(showRoiAction_);
  manToolbar_->addAction(showFrameProcessorAction_);
  manToolbar_->addAction(showMtfControlAction_);

  ///////////////////////////////////////////////////////////////////
}


void MainWindow::setupImagerSettings()
{
  addDockWidget(Qt::RightDockWidgetArea,
      imagerSettingsDock_ =
          new QCameraControlDock("Camera controls", this,
              imagerSettings_ctl = new QImagerSettingsWidget(this)));

  imagerSettingsDock_->setObjectName("imagerSettingsDock_");

  menuView_->addAction(showCameraControlsAction_ =
      imagerSettingsDock_->toggleViewAction());

  imagerSettings_ctl->setCameraWriter(&cameraWriter_);

  connect(imagerSettings_ctl, &QImagerSettingsWidget::selectedCameraChanged,
      [this]() {

        const QImagingCamera::sptr & camera =
            imagerSettings_ctl->selectedCamera();

        centralDisplay_->setCamera(camera);
        focusMeasureThread_->setCamera(camera);
      });


  statusBar()->addWidget(statistics_ctl = new QLabel("Stats", this));
  statistics_ctl->setText("");
  //statistics_ctl->setFrameShape(QFrame::Panel);

  connect(&cameraWriter_, &QCameraWriter::statisticsUpdate,
      this, &ThisClass:: onCameraWriterStatisticsUpdate,
      Qt::QueuedConnection);


}

void MainWindow::setupFrameProcessorControls()
{
  frameProcessorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "frameProcessorDock_",
          "Processing options...",
          frameProcessor_ctl = new QCameraFrameProcessorSelector(this),
          menuView_);

  frameProcessorDock_->hide();

  showFrameProcessorAction_ = frameProcessorDock_->toggleViewAction();
  showFrameProcessorAction_->setIcon(getIcon(ICON_display));
  showFrameProcessorAction_->setToolTip("Show / Hide video frame processing options");


  connect(frameProcessor_ctl, &QImageProcessorSelector::parameterChanged,
      [this]() {
        centralDisplay_->setFrameProcessor(frameProcessor_ctl->current_processor());
      });


  if ( (showdisplayFrameProcessorSettingsAction_  = frameProcessor_ctl->showDisplaysSettingsAction()) ) {

    showdisplayFrameProcessorSettingsAction_->setCheckable(true);
    showdisplayFrameProcessorSettingsAction_->setChecked(false);
    showdisplayFrameProcessorSettingsAction_->setEnabled(true);

    connect(showdisplayFrameProcessorSettingsAction_, &QAction::triggered,
        this, &ThisClass::onShowDisplayFrameProcessorSettingsActionTriggered);
  }


}

void MainWindow::onCameraWriterStatisticsUpdate()
{
  const int cdrops = cameraWriter_.camera() ?
      cameraWriter_.camera()->drops() : 0;

  const int wdrops =
      cameraWriter_.num_dropped_frames();

  statistics_ctl->setText(QString("/round: %1 /time: %2 s /frames: %3 /drops: %4:%5")
      .arg(cameraWriter_.round())
      .arg(cvRound(cameraWriter_.capture_duration() / 1000))
      .arg(cameraWriter_.num_saved_frames())
      .arg(wdrops)
      .arg(cdrops));
}


void MainWindow::setupFocusGraph()
{
  focusMeasureThread_ =
      new QCameraFocusMeasureThread(this);

  focusGraphDock_ =
      addDock<QFocusGraphDock>(this,
          Qt::RightDockWidgetArea,
          "focusGraphDock_",
          "Focus Graph",
          focusGraph_ = new QFocusGraph(this),
          menuView_);

  focusGraphDock_->hide();

  focusGraph_->setFocusMeasureThread(focusMeasureThread_);
  focusMeasureThread_->setRoi(centralDisplay_->roi());

  connect(centralDisplay_, &QCameraFrameDisplay::roiChanged,
      [this](const QRect & rc) {

        focusMeasureThread_->setRoi(rc);

        mousepos_ctl->setText(QString("ROI: x= %1 y= %2 w= %3 h= %4")
            .arg(rc.x())
            .arg(rc.y())
            .arg(rc.width())
            .arg(rc.height()));
      });

}

void MainWindow::setupIndigoFocuser()
{
#if HAVE_INDIGO

  static bool inidigo_initialized = false;
  if ( !inidigo_initialized ) {

    if ( !indigoClient_ ) {
      indigoClient_ = new QIndigoClient("SerImager", this);
    }

    /* This shall be set only before connecting */
    indigo_use_host_suffix = true;
    indigo_set_log_level(INDIGO_LOG_INFO);

    indigo_result status;

    if( (status = indigoClient_->start()) != INDIGO_OK ) {
      QMessageBox::critical(this, "ERROR",
          "indigoClient_.start() fails");
    }
    else if( (status = indigoClient_->load_driver("indigo_focuser_focusdreampro")) ) {
      QMessageBox::critical(this, "ERROR",
          "indigoClient_.loadIndigoDriver(indigo_focuser_focusdreampro) fails");
    }

    inidigo_initialized = true;

    indigoFocuserDock_ =
        addCustomDock(this,
            Qt::RightDockWidgetArea,
            "indigoFocuserDock",
            "Indigo focuser",
            indigoFocuser_ = new QIndigoFocuserWidget(this),
            menuView_);

    indigoFocuser_->setIndigoClient(indigoClient_);
    indigoFocuserDock_->hide();

  }
#endif // HAVE_INDIGO
}

void MainWindow::onShowDisplayFrameProcessorSettingsActionTriggered(bool checked)
{
  if( checked ) {
    if( !displayFrameProcessorSettingsDialogBox_ ) {

      displayFrameProcessorSettingsDialogBox_ = new QDisplayFrameProcessorSettingsDialogBox(this);
      displayFrameProcessorSettingsDialogBox_->setDisplay(centralDisplay_);

      connect(displayFrameProcessorSettingsDialogBox_, &QDisplayFrameProcessorSettingsDialogBox::visibilityChanged,
          [this](bool visible) {
            if ( showdisplayFrameProcessorSettingsAction_ ) {
              showdisplayFrameProcessorSettingsAction_->setChecked(visible);
            }
          });
    }

    displayFrameProcessorSettingsDialogBox_->show();
  }
  else if( displayFrameProcessorSettingsDialogBox_ ) {
    displayFrameProcessorSettingsDialogBox_->hide();
  }

}

void MainWindow::onShowMtfControlActionTriggered(bool checked)
{
  if ( checked && !mtfControl_ ) {

    mtfControl_ = new QMtfControlDialogBox(this);
    mtfControl_->setMtfDisplaySettings(centralDisplay_->mtfDisplayFunction());

    connect(mtfControl_, &QMtfControlDialogBox::visibilityChanged,
        showMtfControlAction_, &QAction::setChecked);

  }

  mtfControl_->setVisible(checked);
}


} /* namespace qserimager */