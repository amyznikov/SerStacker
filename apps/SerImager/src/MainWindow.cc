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

QWidget* addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}

QWidget* createStretch()
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return stretch;
}

}

MainWindow::MainWindow(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(":/qserimager/icons/app-icon.png"));

  setCentralWidget(centralDisplay_ = new QCameraFrameDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  setupStatusbar();
  setupMainMenu();
  setupFocusGraph();
  setupIndigoFocuser();
  setupCameraControls();
  setupDisplayProcessingControls();
  setupMainToolbar();

  restoreState();

  QApplication::instance()->installEventFilter(this);

  connect(centralDisplay_, &QCameraFrameDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mousepos_ctl->setText(centralDisplay_->statusStringForPixel(e->pos()));
        mousepos_ctl->show();
      });

  connect(centralDisplay_, &QCameraFrameDisplay::onMouseLeaveEvent,
      [this](QEvent * e) {
        mousepos_ctl->hide();
      });

  connect(centralDisplay_, &QCameraFrameDisplay::roiChanged,
      [this](const QRect & rc) {

        const QImagingCamera::sptr & currentCamera = cameraControls_ctl->selectedCamera();
        if ( currentCamera ) {
          currentCamera->setRoi(rc);
        }

        mousepos_ctl->setText(QString("ROI: x= %1 y= %2 w= %3 h= %4")
            .arg(rc.x())
            .arg(rc.y())
            .arg(rc.width())
            .arg(rc.height()));
      });

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      [this]() {
        const QImagingCamera::sptr & currentCamera = cameraControls_ctl->selectedCamera();
        if ( currentCamera ) {
          currentCamera->setRoi(centralDisplay_->roi());
        }
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

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

  sb->addWidget(exposure_status_ctl = new QLabel(this));
  sb->addWidget(mousepos_ctl = new QLabel(this));
  sb->addPermanentWidget(capture_status_ctl = new QLabel("", this));
}

void MainWindow::setupCameraControls()
{
  addDockWidget(Qt::RightDockWidgetArea,
      cameraControlsDock_ =
          new QImagingCameraControlsDock("Camera controls", this,
              cameraControls_ctl = new QImagingCameraControlsWidget(this)));

  cameraControlsDock_->setObjectName("imagerSettingsDock_");

  menuView_->addAction(showCameraControlsAction_ =
      cameraControlsDock_->toggleViewAction());

  cameraControls_ctl->setCameraWriter(&cameraWriter_);

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      [this]() {

        const QImagingCamera::sptr & camera =
        cameraControls_ctl->selectedCamera();

        centralDisplay_->setCamera(camera);
        focusMeasureThread_->setCamera(camera);

        if ( camera ) {
          connect(camera.get(), &QImagingCamera::exposureStatusUpdate,
              this, &ThisClass::onExposureStatusUpdate,
              Qt::QueuedConnection);
        }
      });

  connect(&cameraWriter_, &QCameraWriter::statusUpdate,
      this, &ThisClass::onCameraWriterStatusUpdate,
      Qt::QueuedConnection);

}

void MainWindow::setupDisplayProcessingControls()
{
  frameProcessorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "frameProcessorDock_",
          "Display Processing Options",
          frameProcessor_ctl = new QCameraFrameProcessorSelector(this),
          menuView_);

  frameProcessorDock_->hide();

  showFrameProcessorAction_ = frameProcessorDock_->toggleViewAction();
  showFrameProcessorAction_->setIcon(getIcon(ICON_display));
  showFrameProcessorAction_->setToolTip("Show / Hide display frame processing options");

  connect(frameProcessor_ctl, &QImageProcessorSelector::parameterChanged,
      [this]() {
        centralDisplay_->setFrameProcessor(frameProcessor_ctl->current_processor());
      });

  if( (showdisplayFrameProcessorSettingsAction_ = frameProcessor_ctl->showDisplaysSettingsAction()) ) {

    showdisplayFrameProcessorSettingsAction_->setCheckable(true);
    showdisplayFrameProcessorSettingsAction_->setChecked(false);
    showdisplayFrameProcessorSettingsAction_->setEnabled(true);

    connect(showdisplayFrameProcessorSettingsAction_, &QAction::triggered,
        this, &ThisClass::onShowDisplayFrameProcessorSettingsActionTriggered);
  }
}

void MainWindow::onCameraWriterStatusUpdate()
{
  const int capture_drops = cameraWriter_.camera() ?
      cameraWriter_.camera()->drops() : 0;

  const int write_drops =
      cameraWriter_.num_dropped_frames();

  capture_status_ctl->setText(QString("/Round: %1 /time: %2 s /frames: %3 /drops: %4:%5")
      .arg(cameraWriter_.round())
      .arg(cvRound(cameraWriter_.capture_duration() / 1000))
      .arg(cameraWriter_.num_saved_frames())
      .arg(write_drops)
      .arg(capture_drops));
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
}

void MainWindow::setupIndigoFocuser()
{
#if HAVE_INDIGO

  static bool inidigo_initialized = false;
  if( !inidigo_initialized ) {

    if( !indigoClient_ ) {
      indigoClient_ =
          new QIndigoClient("SerImager", this);
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
  if( checked && !mtfControl_ ) {

    mtfControl_ = new QMtfControlDialogBox(this);
    mtfControl_->setMtfDisplaySettings(centralDisplay_->mtfDisplayFunction());

    connect(mtfControl_, &QMtfControlDialogBox::visibilityChanged,
        showMtfControlAction_, &QAction::setChecked);
  }

  mtfControl_->setVisible(checked);
}

void MainWindow::onExposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed)
{
  switch (status) {
    case QImagingCamera::Exposure_working:
      exposure_status_ctl->setText(QString::asprintf("Exp: %.1f / %.1f [s]", elapsed * 1e-3, exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_success:
      exposure_status_ctl->setText(QString::asprintf("Exp: %.1f [s] OK", exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_failed:
      exposure_status_ctl->setText(QString::asprintf("Exp: %.1f [s] FAILED", exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_idle:
      exposure_status_ctl->setText("");
      exposure_status_ctl->hide();
      break;
  }
}

} /* namespace serimager */
