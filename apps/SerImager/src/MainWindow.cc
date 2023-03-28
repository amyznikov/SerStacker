/*
 * MainWindow.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include "camera/ffmpeg/QFFStreams.h"
#include "pipeline/QLiveCameraCalibration/QLiveCameraCalibrationPipeline.h"
#include "pipeline/QLiveStereoCalibration/QLiveStereoCalibrationPipeline.h"
#include "pipeline/QLiveRegularStereo/QLiveRegularStereoPipeline.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>

namespace serimager {

#define ICON_histogram        ":/serimager/icons/histogram.png"
#define ICON_display          ":/serimager/icons/display.png"
#define ICON_shapes           ":/serimager/icons/shapes.png"
#define ICON_roi              ":/serimager/icons/roi.png"
#define ICON_line             ":/serimager/icons/line.png"
#define ICON_target           ":/serimager/icons/target.png"
#define ICON_process          ":/serimager/icons/process.png"

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

template<typename Slot>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip, Slot && slot)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  QObject::connect(action, &QAction::triggered, slot);

  return action;
}

QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  return action;
}

QToolButton* createToolButtonWithPopupMenu(QAction * defaultAction, QMenu * menu)
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

} // namespace

MainWindow::MainWindow(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(":/serimager/icons/app-icon.png"));

  setCentralWidget(centralDisplay_ = new QVideoFrameDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  QFFStreams::load();
  liveView_ = new QLivePipelineThread(this);
  liveView_->setDisplay(centralDisplay_);


  setupStatusbar();
  setupMainMenu();
  setupLogWidget();
  setupIndigoFocuser();
  setupCameraControls();
  setupFocusGraph();
  setupLivePipelineControls();
  setupDisplayProcessingControls();
  setupShapeOptions();
  setupMainToolbar();

  restoreState();

  QApplication::instance()->installEventFilter(this);

  connect(centralDisplay_, &QVideoFrameDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mousepos_ctl->setText(centralDisplay_->statusStringForPixel(e->pos()));
        mousepos_ctl->show();
      });

  connect(centralDisplay_, &QVideoFrameDisplay::onMouseLeaveEvent,
      [this](QEvent * e) {
        mousepos_ctl->hide();
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

  menuViewShapes_ =
      menuView_->addMenu(getIcon(ICON_shapes),
          "Shapes");

  menuViewShapes_->addAction(showRectShapeAction_ =
      createCheckableAction(getIcon(ICON_roi),
          "ROI Rectangle",
          "Show / Hide ROI rectangle",
          [this](bool checked) {
            centralDisplay_->rectShape()->setVisible(checked);
          }));


  menuViewShapes_->addAction(showLineShapeAction_ =
      createCheckableAction(getIcon(ICON_line),
          "Line Shape",
          "Show / Hide Line Shape",
          [this](bool checked) {
            centralDisplay_->lineShape()->setVisible(checked);
          }));


  menuViewShapes_->addAction(showTargetShapeAction_ =
      createCheckableAction(getIcon(ICON_target),
          "Target Shape",
          "Show / Hide Target Shape",
          [this](bool checked) {
            centralDisplay_->targetShape()->setVisible(checked);
          }));


  showRectShapeAction_->setChecked(centralDisplay_->rectShape()->isVisible());
  showLineShapeAction_->setChecked(centralDisplay_->lineShape()->isVisible());
  showTargetShapeAction_->setChecked(centralDisplay_->targetShape()->isVisible());


  /////////////////////////////////////

  menuView_->addAction(showMtfControlAction_ =
      createCheckableAction(getIcon(ICON_histogram),
          "Display Options...",
          "Show / Hide Display Options",
          this,
          &ThisClass::onShowMtfControlActionTriggered));

  /////////////////////////////////////


}

void MainWindow::setupShapeOptions()
{
  QAction * action;

  //
  // Rect (ROI) shape
  //
  rectShapeOptionsDialogBox_ =
      new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
          centralDisplay_->rectShape(),
          this);

  rectShapeActionsMenu_.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure ROI rectangle options",
          [this](bool checked) {
            rectShapeOptionsDialogBox_->setVisible(checked);
          }));

  connect(rectShapeOptionsDialogBox_, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          centralDisplay_->rectShape()->setVisible(true);
          showRectShapeAction_->setChecked(true);
        }
      });

  connect(centralDisplay_->rectShape(), &QGraphicsShape::itemChanged,
      [this]() {

        QGraphicsRectShape * shape =
            centralDisplay_->rectShape();

        const QRectF rc =
            shape->sceneRect();

        const QImagingCamera::sptr & currentCamera =
            cameraControls_ctl->selectedCamera();

        if ( currentCamera ) {
          currentCamera->setRoi(QRect(rc.x(), rc.y(),
                  rc.width(), rc.height()));
        }

        mousepos_ctl->setText(qsprintf("ROI: x= %g y= %g size= [%g x %g] center= (%g %g)",
                rc.x(), rc.y(), rc.width(), rc.height(), rc.center().x(), rc.center().y() ));
      });

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      [this]() {

        const QImagingCamera::sptr & currentCamera =
            cameraControls_ctl->selectedCamera();

        if ( currentCamera ) {

          const QRectF rc =
              centralDisplay_->rectShape()->sceneRect();

          currentCamera->setRoi(QRect(rc.x(), rc.y(),
                  rc.width(), rc.height()));
        }
      });


  //
  // Target shape
  //

  targetShapeOptionsDialogBox_ =
      new QGraphicsTargetShapeSettingsDialogBox("Target shape options",
          centralDisplay_->targetShape(),
          this);

  targetShapeActionsMenu_.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure target shape options",
          [this](bool checked) {
            targetShapeOptionsDialogBox_->setVisible(checked);
          }));

  connect(targetShapeOptionsDialogBox_, &QGraphicsTargetShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          centralDisplay_->targetShape()->setVisible(true);
          showTargetShapeAction_->setChecked(true);
        }
      });


  connect(centralDisplay_->targetShape(), &QGraphicsShape::itemChanged,
      [this]() {

        const QGraphicsTargetShape * shape =
            centralDisplay_->targetShape();

        const QPointF cp =
            shape->mapToScene(shape->center());

        mousepos_ctl->setText(qsprintf("Center: (%g %g)", cp.x(), cp.y()));
      });

  //
  // Line shape
  //
  lineShapeOptionsDialogBox_ =
      new QGraphicsLineShapeSettingsDialogBox("Line shape options",
          centralDisplay_->lineShape(),
          this);

  lineShapeActionsMenu_.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure line shape options",
          [this](bool checked) {
            lineShapeOptionsDialogBox_->setVisible(checked);
          }));

  connect(lineShapeOptionsDialogBox_, &QGraphicsLineShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          centralDisplay_->lineShape()->setVisible(true);
          showLineShapeAction_->setChecked(visible);
        }
      });


  connect(centralDisplay_->lineShape(), &QGraphicsShape::itemChanged,
      [this]() {

        QGraphicsLineShape * shape =
            centralDisplay_->lineShape();

        const QLineF line = shape->sceneLine();

        const QPointF p1 = line.p1();
        const QPointF p2 = line.p2();
        const double length = hypot(p2.x()-p1.x(), p2.y()-p1.y());
        const double angle = atan2(p2.y()-p1.y(), p2.x()-p1.x());

        mousepos_ctl->setText(qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
                p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));
      });

}

void MainWindow::setupMainToolbar()
{
  menuBar()->setCornerWidget(manToolbar_ =
      new QToolBar(this),
      Qt::TopRightCorner);

  manToolbar_->setContentsMargins(0, 0, 0, 0);
  manToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  manToolbar_->setIconSize(QSize(32, 18));

  ///////////////////////////////////////////////////////////////////

  manToolbar_->addWidget(rectShapeActionsButton_ =
      createToolButtonWithPopupMenu(showRectShapeAction_,
          &rectShapeActionsMenu_));

  manToolbar_->addWidget(lineShapeActionsButton_ =
      createToolButtonWithPopupMenu(showLineShapeAction_,
          &lineShapeActionsMenu_));

  manToolbar_->addWidget(targetShapeActionsButton_ =
      createToolButtonWithPopupMenu(showTargetShapeAction_,
          &targetShapeActionsMenu_));




  ///////////////////////////////////////////////////////////////////


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

void MainWindow::setupLogWidget()
{
  logwidgetDock_ =
      addCustomDock(this,
          Qt::BottomDockWidgetArea,
          "logwidgetDock_",
          "Debug log",
          logwidget_ctl = new QLogWidget(this),
          menuView_);

  logwidgetDock_->hide();
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

        liveView_->setCamera(camera);
        focusMeasure_->setCamera(camera);

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

void MainWindow::setupLivePipelineControls()
{
  pipelineCollection_.addPipelineType("CameraCalibration",
      "LIve Camera Calibration",
      [](const QString & name) {
        return new QLiveCameraCalibrationPipeline(name);
      });

  pipelineCollection_.addPipelineType("StereoCalibration",
      "LIve Stereo Camera Calibration",
      [](const QString & name) {
        return new QLiveStereoCalibrationPipeline(name);
      });

  pipelineCollection_.addPipelineType("RegularStereo",
      "LIve Stereo Matching",
      [](const QString & name) {
        return new QLiveRegularStereoPipeline(name);
      });



  pipelineSelectorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "pipelineSelectorDock_",
          "Live Pipelines",
          pipelineSelector_ctl = new QLivePipelineSelectionWidget(this),
          menuView_);

  pipelineSelectorDock_->hide();

  showPipelineSelectorAction_ = pipelineSelectorDock_->toggleViewAction();
  showPipelineSelectorAction_->setIcon(getIcon(ICON_process));
  showPipelineSelectorAction_->setToolTip("Show / Hide live pipelines");

  pipelineCollection_.load();
  pipelineSelector_ctl->setPipelineCollection(&pipelineCollection_);
  pipelineSelector_ctl->setLiveThread(liveView_);

  connect(pipelineSelector_ctl, &QLivePipelineSelectionWidget::parameterChanged,
      [this]() {
        // centralDisplay_->setFrameProcessor(frameProcessor_ctl->current_processor());
      });

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

  if( (showLiveThreadSettingsDialogBoxAction_ = frameProcessor_ctl->showDisplaysSettingsAction()) ) {

    showLiveThreadSettingsDialogBoxAction_->setCheckable(true);
    showLiveThreadSettingsDialogBoxAction_->setChecked(false);
    showLiveThreadSettingsDialogBoxAction_->setEnabled(true);

    connect(showLiveThreadSettingsDialogBoxAction_, &QAction::triggered,
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
  focusMeasure_ =
      new QCameraFocusMeasure(this);

  focusGraphDock_ =
      addDock<QFocusGraphDock>(this,
          Qt::RightDockWidgetArea,
          "focusGraphDock_",
          "Focus Graph",
          focusGraph_ = new QFocusGraph(this),
          menuView_);

  focusGraphDock_->hide();
  focusGraph_->setFocusMeasureProvider(focusMeasure_);
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
    if( !liveThreadSettingsDialogBox_ ) {

      liveThreadSettingsDialogBox_ = new QLiveThreadSettingsDialogBox(this);
      liveThreadSettingsDialogBox_->setLiveThread(liveView_);

      connect(liveThreadSettingsDialogBox_, &QLiveThreadSettingsDialogBox::visibilityChanged,
          [this](bool visible) {
            if ( showLiveThreadSettingsDialogBoxAction_ ) {
              showLiveThreadSettingsDialogBoxAction_->setChecked(visible);
            }
          });
    }

    liveThreadSettingsDialogBox_->show();
  }
  else if( liveThreadSettingsDialogBox_ ) {
    liveThreadSettingsDialogBox_->hide();
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
