/*
 * MainWindow.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include "camera/ffmpeg/QFFStreams.h"
#include "pipeline/QLiveCameraCalibration/QLiveCameraCalibrationOptions.h"
#include "pipeline/QLiveStereoCalibration/QLiveStereoCalibrationOptions.h"
#include "pipeline/QLiveRegularStereo/QLiveRegularStereoOptions.h"
#include "pipeline/QLiveImageProcessingPipeline/QLiveImageProcessingOptions.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>

namespace serimager {

#define ICON_camera           ":/serimager/icons/camera.png"
#define ICON_histogram        ":/serimager/icons/histogram.png"
#define ICON_shapes           ":/serimager/icons/shapes.png"
#define ICON_roi              ":/serimager/icons/roi.png"
#define ICON_line             ":/serimager/icons/line.png"
#define ICON_target           ":/serimager/icons/target.png"
#define ICON_process          ":/serimager/icons/process.png"
#define ICON_log              ":/serimager/icons/log.png"
#define ICON_bayer            ":/gui/icons/bayer.png"

#define ICON_measures         ":/qmeasure/icons/measure.png"


MainWindow::MainWindow(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(":/serimager/icons/app-icon.png"));

  setCentralWidget(centralDisplay_ = new QLiveDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  QFFStreams::load();
  liveView_ = new QLivePipelineThread(this);
  liveView_->setDisplay(centralDisplay_);


  setupMainMenu();
  setupLogWidget();
  setupIndigoFocuser();
  setupCameraControls();
  setupMeasures();
  setupImageProcessingControls();
  setupLivePipelineControls();
  setupShapeOptions();
  setupMainToolbar();
  setupStatusbar();

  restoreState();

  connect(centralDisplay_, &QImageViewer::currentImageChanged,
      this, &ThisClass::onCurrentImageChanged);

  connect(centralDisplay_, &QLiveDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mousepos_ctl->setText(centralDisplay_->statusStringForPixel(e->pos()));
        mousepos_ctl->show();
      });

  connect(centralDisplay_, &QLiveDisplay::onMouseLeaveEvent,
      [this](QEvent * e) {
        mousepos_ctl->hide();
      });

}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);
}

void MainWindow::setupMainMenu()
{
  Base::setupMainMenu();

  ///////////////////////////////////////////////////////////////////

  menuFile_->addSeparator();
  menuFile_->addAction("Quit", [this]() {
    close();
  });

  ///////////////////////////////////////////////////////////////////

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

        updateMeasurements();
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


  manToolbar_->addAction(showImageProcessorAction_);
  manToolbar_->addAction(showMtfControlAction_);

  ///////////////////////////////////////////////////////////////////

  manToolbar_->addWidget(displayScaleControl_ = new QScaleSelectionButton());
  displayScaleControl_->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(displayScaleControl_, &QScaleSelectionButton::scaleChanged,
      [this](int currentScale) {
        centralDisplay_->setViewScale(currentScale);
      });


  ///////////////////////////////////////////////////////////////////

  manToolbar_->addAction(showLiveThreadSettingsAction_ =
      createCheckableAction(getIcon(ICON_bayer), "Bayer", "Configure debayer options",
          this, &ThisClass::onShowLiveThreadSettingsActionTriggered));

  ///////////////////////////////////////////////////////////////////

  //manToolbar_->addAction(showMeasureDisplayDialogBoxAction_);

  manToolbar_->addWidget(measureActionsToolButton_ =
      createToolButtonWithMenu(getIcon(ICON_measures),
          "Measures",
          "Measures menu",
          &measuresMenu_));

  ///////////////////////////////////////////////////////////////////
}

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

  sb->addWidget(show_log_ctl = new QToolButton());
  sb->addWidget(exposure_status_ctl = new QLabel(this));
  sb->addWidget(mousepos_ctl = new QLabel(this));
  sb->addPermanentWidget(capture_status_ctl = new QLabel("", this));

  show_log_ctl->setDefaultAction(showLogWidgetAction_);
}

void MainWindow::setupCameraControls()
{
  addDockWidget(Qt::RightDockWidgetArea,
      cameraControlsDock_ =
          new QImagingCameraControlsDock("Camera controls", this,
              cameraControls_ctl = new QImagingCameraControlsWidget(this)));

  cameraControlsDock_->setObjectName("imagerSettingsDock_");
  cameraControlsDock_->titleBar()->setWindowIcon(getIcon(ICON_camera));

  menuView_->addAction(showCameraControlsAction_ =
      cameraControlsDock_->toggleViewAction());

  showCameraControlsAction_->setIcon(getIcon(ICON_camera));

  cameraControls_ctl->setCameraWriter(&cameraWriter_);

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      [this]() {

        const QImagingCamera::sptr & camera =
            cameraControls_ctl->selectedCamera();

        liveView_->setCamera(camera);

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
  pipelineCollection_.addPipelineClassFactory(
      QLiveImageProcessingPipeline::className(),
      "Generic image processing",
      [](const QString & name) {
        return new QLiveImageProcessingPipeline(name);
      },
      [](QWidget * parent) {
        return new QLiveImageProcessingOptions(parent);
      });


  pipelineCollection_.addPipelineClassFactory(
      QLiveCameraCalibrationPipeline::className(),
      "Live Camera Calibration",
      [](const QString & name) {
        return new QLiveCameraCalibrationPipeline(name);
  },
  [](QWidget * parent) {
    return new QLiveCameraCalibrationOptions(parent);
  });


  pipelineCollection_.addPipelineClassFactory(
      QLiveStereoCalibrationPipeline::className(),
      "Live Stereo Camera Calibration",
      [](const QString & name) {
        return new QLiveStereoCalibrationPipeline(name);
  },
  [](QWidget * parent) {
    return new QLiveStereoCalibrationOptions(parent);
  });


  pipelineCollection_.addPipelineClassFactory(
      QLiveRegularStereoPipeline::className(),
      "Live Stereo Matching",
      [](const QString & name) {
        return new QLiveRegularStereoPipeline(name);
  },
  [](QWidget * parent) {
    return new QLiveRegularStereoOptions(parent);
  });



  pipelineSelectorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "pipelineSelectorDock_",
          "Live Pipelines",
          pipelineSelector_ctl = new QLivePipelineSelectionWidget(this),
          menuView_);

  pipelineSelectorDock_->titleBar()->setWindowIcon(getIcon(ICON_process));


  pipelineSelectorDock_->hide();

  showPipelineSelectorAction_ = pipelineSelectorDock_->toggleViewAction();
  showPipelineSelectorAction_->setIcon(getIcon(ICON_process));
  showPipelineSelectorAction_->setToolTip("Show / Hide live pipelines");

  pipelineCollection_.load();
  pipelineSelector_ctl->setPipelineCollection(&pipelineCollection_);
  pipelineSelector_ctl->setLiveThread(liveView_);
}

void MainWindow::onImageProcessorParameterChanged()
{
  centralDisplay_->setFrameProcessor(imageProcessor_ctl->current_processor());
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  if( visible ) {
    mtfControl_->setMtfDisplaySettings(centralDisplay_->mtfDisplayFunction());
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


void MainWindow::onCurrentImageChanged()
{
  updateMeasurements();
}

void MainWindow::updateMeasurements()
{
  if( !QMeasureProvider::requested_measures().empty() && centralDisplay_->rectShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(centralDisplay_);

    QMeasureProvider::compute(centralDisplay_->currentImage(),
        centralDisplay_->currentMask(),
        centralDisplay_->rectShape()->iSceneRect());
  }
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

void MainWindow::onShowLiveThreadSettingsActionTriggered(bool checked)
{
  if( checked ) {
    if( !liveThreadSettingsDialogBox_ ) {

      liveThreadSettingsDialogBox_ = new QLiveThreadSettingsDialogBox(this);
      liveThreadSettingsDialogBox_->setLiveThread(liveView_);

      connect(liveThreadSettingsDialogBox_, &QLiveThreadSettingsDialogBox::visibilityChanged,
          [this](bool visible) {
            if ( showLiveThreadSettingsAction_ ) {
              showLiveThreadSettingsAction_->setChecked(visible);
            }
          });
    }

    liveThreadSettingsDialogBox_->show();
  }
  else if( liveThreadSettingsDialogBox_ ) {
    liveThreadSettingsDialogBox_->hide();
  }

}

//void MainWindow::onShowMtfControlActionTriggered(bool checked)
//{
//  if( checked && !mtfControl_ ) {
//
//    mtfControl_ = new QMtfControlDialogBox(this);
//    mtfControl_->setMtfDisplaySettings(centralDisplay_->mtfDisplayFunction());
//
//    connect(mtfControl_, &QMtfControlDialogBox::visibilityChanged,
//        showMtfControlAction_, &QAction::setChecked);
//  }
//
//  mtfControl_->setVisible(checked);
//}
//
//void MainWindow::onShowMeasureSettingsActionTriggered(bool checked)
//{
//  if ( !checked ) {
//    if ( measureSettingsDisplay_ ) {
//      measureSettingsDisplay_->hide();
//    }
//  }
//  else {
//    if ( !measureSettingsDisplay_ ) {
//
//      measureSettingsDisplay_ = new QMeasureSettingsDialogBox(this);
//      // measureSettingsDisplay_->resize(QApplication::primaryScreen()->geometry().size() / 2);
//
//      connect(measureSettingsDisplay_, &QMeasureSettingsDialogBox::visibilityChanged,
//          showMeasureSettingsAction_, &QAction::setChecked);
//    }
//
//    measureSettingsDisplay_->show();
//    measureSettingsDisplay_->raise();
//    measureSettingsDisplay_->setFocus();
//  }
//}
//
//void MainWindow::onShowMeasureDisplayActionTriggered(bool checked)
//{
//  if ( !checked ) {
//    if ( measureDisplay_ ) {
//      measureDisplay_->hide();
//    }
//  }
//  else {
//    if ( !measureDisplay_ ) {
//
//      measureDisplay_ = new QMeasureDisplayDialogBox(this);
//      measureDisplay_->resize(QApplication::primaryScreen()->geometry().size() / 2);
//
//      connect(measureDisplay_, &QMeasureDisplayDialogBox::visibilityChanged,
//          showMeasureDisplayAction_, &QAction::setChecked);
//    }
//
//    measureDisplay_->show();
//    measureDisplay_->raise();
//    measureDisplay_->setFocus();
//  }
//}

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
