/*
 * MainWindow.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/AboutDialog.h>
#include "camera/ffmpeg/QFFStreams.h"
#include "camera/libcamera-sctp/QLCSCTPStreams.h"
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>

namespace serimager {

#define APP_ICON              ":/serimager/icons/app-icon.png"
#define ICON_camera           ":/serimager/icons/camera.png"
#define ICON_histogram        ":/serimager/icons/histogram.png"
#define ICON_shapes           ":/serimager/icons/shapes.png"
#define ICON_roi              ":/serimager/icons/roi.png"
#define ICON_line             ":/serimager/icons/line.png"
#define ICON_target           ":/serimager/icons/target.png"
#define ICON_process          ":/serimager/icons/process.png"
#define ICON_log              ":/serimager/icons/log.png"
#define ICON_bayer            ":/gui/icons/bayer.png"
#define ICON_copy             ":/gui/icons/copy.png"

#define ICON_measures         ":/qmeasure/icons/measure.png"

namespace  {

inline bool is_visible(QWidget * w)
{
  return w && w->isVisible();
}

}  // namespace


MainWindow::MainWindow(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(APP_ICON));

  QImageProcessorsCollection::load();

  setCentralWidget(centralDisplay_ = new QLiveDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  QFFStreams::load();
#if HAVE_QLCSCTPCamera
  QLCSCTPStreams::load();
#endif //HAVE_QLCSCTPCamera

  liveView_ = new QLivePipelineThread(this);
  liveView_->setDisplay(centralDisplay_);


  setupMainMenu();
  setupLogWidget();
  setupIndigoFocuser();
  setupCameraControls();
  setupMeasures();
  setupImageProcessingControls();
  setupPipelines();
  setupShapeOptions();
  setupMainToolbar();
  setupStatusbar();
  setupProfileGraph();
  setupDisplayImageVideoWriter();

  restoreState();

  connect(centralDisplay_, &QImageViewer::currentImageChanged,
      this, &ThisClass::onCurrentImageChanged);

  connect(centralDisplay_, &QImageViewer::displayImageChanged,
      this, &ThisClass::onCurrentDisplayImageChanged);


  connect(centralDisplay_, &QLiveDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mouse_status_ctl->setText(centralDisplay_->statusStringForPixel(e->pos()));
        mouse_status_ctl->show();
      });

  connect(centralDisplay_, &QLiveDisplay::onMouseLeaveEvent,
      [this](QEvent * e) {
        if ( mouse_status_ctl->isVisible() ) {
          mouse_status_ctl->hide();
        }
      });

  //
  // Add to the end of View menu
  //
  menuView->addSeparator();
  menuView->addAction("About SerImager...",
      [this]() {
        AboutDialog dialog("SerImager", getPixmap(APP_ICON), this);
        dialog.exec();
      });
}

MainWindow::~MainWindow()
{
  saveState();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
  Base::closeEvent(event);

  if ( event->isAccepted() ) {
  }

}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);

  if ( imageProcessor_ctl ) {

    settings.setValue(QString("imageProcessor/selected_processor"),
        imageProcessor_ctl->selected_processor());
  }

  if ( dataframeProcessor_ctl ) {

    settings.setValue(QString("dataframeProcessor/selected_processor"),
        dataframeProcessor_ctl->selected_processor());
  }

  if ( cameraControls_ctl ) {
    cameraControls_ctl->saveSettings(settings, "QImagingCameraControls");
  }

  diplayImageWriter_.saveSettings(settings, "QDisplayVideoWriter");
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( imageProcessor_ctl ) {

    const QString selected_processor =
        settings.value(QString("imageProcessor/selected_processor")).toString();

    if ( !selected_processor.isEmpty() ) {
      imageProcessor_ctl->set_selected_processor(selected_processor);
    }

  }

  if ( dataframeProcessor_ctl ) {

    const QString selected_processor =
        settings.value(QString("dataframeProcessor/selected_processor")).toString();

    if ( !selected_processor.isEmpty() ) {
      dataframeProcessor_ctl->set_selected_processor(selected_processor);
    }
  }

  if ( cameraControls_ctl ) {
    cameraControls_ctl->loadSettings(settings, "QImagingCameraControls");
  }

  diplayImageWriter_.loadSettings(settings, "QDisplayVideoWriter");
}

void MainWindow::setupMainMenu()
{
  Base::setupMainMenu();

  ///////////////////////////////////////////////////////////////////

  menuFile->addSeparator();
  menuFile->addAction("Quit", [this]() {
    close();
  });


  ///////////////////////////////////////////////////////////////////

  menuEdit->addAction(copyDisplayImageAction =
      createAction(getIcon(ICON_copy),
          "Copy display image to clipboard (Ctrl+c)",
          "Copy display image to clipboard (Ctrl+c)",
          [this]() {
            if ( centralDisplay_->rectShape()->isVisible() ) {
              centralDisplay_->copyDisplayImageROIToClipboard(centralDisplay_->rectShape()->iSceneRect());
            }
            else {
              centralDisplay_->copyDisplayImageToClipboard();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              centralDisplay_, nullptr, nullptr,
              Qt::WindowShortcut)));


  menuEdit->addAction(copyDisplayViewportAction =
      createAction(QIcon(),
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          [this]() {
            if ( centralDisplay_->isVisible() ) {
              QPixmap pxmap = centralDisplay_->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
              centralDisplay_, nullptr, nullptr,
              Qt::WindowShortcut)));

  ///////////////////////////////////////////////////////////////////

  menuViewShapes_ =
      menuView->addMenu(getIcon(ICON_shapes),
          "Shapes");

  menuViewShapes_->addAction(showRectShapeAction_ =
      createCheckableAction(getIcon(ICON_roi),
          "ROI Rectangle",
          "Show / Hide ROI rectangle",
          is_visible(centralDisplay_) && centralDisplay_->rectShape()->isVisible(),
          [this](bool checked) {
            centralDisplay_->rectShape()->setVisible(checked);
          }));


  menuViewShapes_->addAction(showLineShapeAction_ =
      createCheckableAction(getIcon(ICON_line),
          "Line Shape",
          "Show / Hide Line Shape",
          is_visible(centralDisplay_) && centralDisplay_->lineShape()->isVisible(),
          [this](bool checked) {
            centralDisplay_->lineShape()->setVisible(checked);
          }));


  menuViewShapes_->addAction(showTargetShapeAction_ =
      createCheckableAction(getIcon(ICON_target),
          "Target Shape",
          "Show / Hide Target Shape",
          is_visible(centralDisplay_) && centralDisplay_->targetShape()->isVisible(),
          [this](bool checked) {
            centralDisplay_->targetShape()->setVisible(checked);
          }));


//  showRectShapeAction_->setChecked(centralDisplay_->rectShape()->isVisible());
//  showLineShapeAction_->setChecked(centralDisplay_->lineShape()->isVisible());
//  showTargetShapeAction_->setChecked(centralDisplay_->targetShape()->isVisible());


  /////////////////////////////////////

  menuView->addAction(showMtfControlAction =
      createCheckableAction(getIcon(ICON_histogram),
          "Display Options...",
          "Show / Hide Display Options",
          is_visible(mtfControl),
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
          is_visible(rectShapeOptionsDialogBox_) && rectShapeOptionsDialogBox_->isVisible(),
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
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  connect(centralDisplay_->rectShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);


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
          is_visible(targetShapeOptionsDialogBox_),
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
      this, &ThisClass::onCentralDisplayTargetShapeChanged);

  connect(centralDisplay_->targetShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayTargetShapeChanged);


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
          is_visible(lineShapeOptionsDialogBox_),
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
      this, &ThisClass::onCentralDisplayLineShapeChanged);

  connect(centralDisplay_->lineShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayLineShapeChanged);

}

void MainWindow::onCentralDisplayROIShapeChanged()
{
  QGraphicsRectShape *shape =
      centralDisplay_->rectShape();

  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      showRectShapeAction_->setChecked(false);
    }
    else {

      const QRectF rc =
          shape->sceneRect();

      const QImagingCamera::sptr &currentCamera =
          cameraControls_ctl->selectedCamera();

      if( currentCamera ) {
        currentCamera->setRoi(QRect(rc.x(), rc.y(),
            rc.width(), rc.height()));
      }

      shape_status_ctl->setText(qsprintf("ROI: x= %g y= %g size= [%g x %g] center= (%g %g)",
          rc.x(), rc.y(), rc.width(), rc.height(), rc.center().x(), rc.center().y()));

      shape_status_ctl->show();
      showRectShapeAction_->setChecked(true);

      updateMeasurements();
    }
  }

}

void MainWindow::onCentralDisplayLineShapeChanged()
{
  QGraphicsLineShape *shape =
      centralDisplay_->lineShape();

  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      showLineShapeAction_->setChecked(false);
    }
    else {

      const QLineF line =
          shape->sceneLine();

      const QPointF p1 = line.p1();
      const QPointF p2 = line.p2();
      const double length = hypot(p2.x() - p1.x(), p2.y() - p1.y());
      const double angle = atan2(p2.y() - p1.y(), p2.x() - p1.x());

      shape_status_ctl->setText(
          qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
              p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));

      shape_status_ctl->show();
      showLineShapeAction_->setChecked(true);

      if( is_visible(profileGraph_ctl) ) {

        QImageViewer::current_image_lock lock(centralDisplay_);

        profileGraph_ctl->showProfilePlot(line,
            centralDisplay_->currentImage(),
            centralDisplay_->currentMask());
      }
    }
  }
}

void MainWindow::onCentralDisplayTargetShapeChanged()
{
  const QGraphicsTargetShape *shape = centralDisplay_->targetShape();
  if( !shape->isVisible() ) {
    shape_status_ctl->hide();
    showTargetShapeAction_->setChecked(false);
  }
  else {
    const QPointF cp = shape->mapToScene(shape->center());
    shape_status_ctl->setText(qsprintf("Center: (%g %g)", cp.x(), cp.y()));
    shape_status_ctl->show();
    showTargetShapeAction_->setChecked(true);
  }
}

void MainWindow::onPlotProfileDialogBoxVisibilityChanged(bool visible)
{
  Base::onPlotProfileDialogBoxVisibilityChanged(visible);

  if( is_visible(profileGraph_ctl) ) {

    QGraphicsLineShape *shape =
        centralDisplay_->lineShape();

    if( shape && !shape->isVisible() ) {
      shape->setVisible(true);
    }
  }
}

void MainWindow::setupMainToolbar()
{
  menuBar()->setCornerWidget(mainToolbar_ =
      new QToolBar(this),
      Qt::TopRightCorner);

  mainToolbar_->setContentsMargins(0, 0, 0, 0);
  mainToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  mainToolbar_->setIconSize(QSize(32, 18));

  ///////////////////////////////////////////////////////////////////

  mainToolbar_->addWidget(rectShapeActionsButton_ =
      createToolButtonWithPopupMenu(showRectShapeAction_,
          &rectShapeActionsMenu_));

  mainToolbar_->addWidget(lineShapeActionsButton_ =
      createToolButtonWithPopupMenu(showLineShapeAction_,
          &lineShapeActionsMenu_));

  mainToolbar_->addWidget(targetShapeActionsButton_ =
      createToolButtonWithPopupMenu(showTargetShapeAction_,
          &targetShapeActionsMenu_));




  ///////////////////////////////////////////////////////////////////


  mainToolbar_->addAction(showImageProcessorAction);
  mainToolbar_->addAction(showMtfControlAction);

  ///////////////////////////////////////////////////////////////////

  mainToolbar_->addWidget(displayScaleControl_ = new QScaleSelectionButton());
  displayScaleControl_->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(displayScaleControl_, &QScaleSelectionButton::scaleChanged,
      [this](int currentScale) {
        centralDisplay_->setViewScale(currentScale);
      });


  ///////////////////////////////////////////////////////////////////

  mainToolbar_->addAction(showLiveThreadSettingsAction_ =
      createCheckableAction(getIcon(ICON_bayer),
          "Bayer",
          "Configure debayer options",
          is_visible(liveThreadSettingsDialogBox_),
          this,
          &ThisClass::onShowLiveThreadSettingsActionTriggered));

  ///////////////////////////////////////////////////////////////////

  //manToolbar_->addAction(showMeasureDisplayDialogBoxAction_);

  mainToolbar_->addWidget(measureActionsToolButton_ =
      createToolButtonWithMenu(getIcon(ICON_measures),
          "Measures",
          "Measures menu",
          &measuresMenu));

  ///////////////////////////////////////////////////////////////////
}

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

  sb->addWidget(show_log_ctl = new QToolButton());
  sb->addWidget(shape_status_ctl = new QLabel(this));
  sb->addWidget(mouse_status_ctl = new QLabel(this));
  sb->addPermanentWidget(exposure_status_ctl = new QLabel(this));
  sb->addPermanentWidget(capture_status_ctl = new QLabel("", this));

  show_log_ctl->setDefaultAction(showLogWidgetAction);
}

void MainWindow::setupCameraControls()
{
  addDockWidget(Qt::RightDockWidgetArea,
      cameraControlsDock_ =
          new QImagingCameraControlsDock("Camera controls", this,
              cameraControls_ctl = new QImagingCameraControlsWidget(this)));

  cameraControlsDock_->setObjectName("imagerSettingsDock_");
  cameraControlsDock_->titleBar()->setWindowIcon(getIcon(ICON_camera));

  menuView->addAction(showCameraControlsAction_ =
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

void MainWindow::setupPipelines()
{
  registerPipelineClasses();

  pipelineSelectorDock_ =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "pipelineSelectorDock_",
          "Live Pipelines",
          pipelineSelector_ctl = new QLivePipelineSelectionWidget(this),
          menuView);

  pipelineSelectorDock_->titleBar()->setWindowIcon(getIcon(ICON_process));


  pipelineSelectorDock_->hide();

  showPipelineSelectorAction_ = pipelineSelectorDock_->toggleViewAction();
  showPipelineSelectorAction_->setIcon(getIcon(ICON_process));
  showPipelineSelectorAction_->setToolTip("Show / Hide live pipelines");

  //pipelineCollection_.load();
  //pipelineSelector_ctl->setPipelineCollection(&pipelineCollection_);
  pipelineSelector_ctl->loadPipelines();
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
    mtfControl->setMtfDisplaySettings(getCurrentMtfDisplay());
  }
}

IMtfDisplay * MainWindow::getCurrentMtfDisplay()
{
  return centralDisplay_->mtfDisplayFunction();
}


void MainWindow::onCameraWriterStatusUpdate()
{
  const int capture_drops = cameraWriter_.camera() ?
      cameraWriter_.camera()->drops() : 0;

  const int write_drops =
      cameraWriter_.num_dropped_frames();

  capture_status_ctl->setText(QString("|R: %1 |T: %2 s |F: %3 |D: %4:%5")
      .arg(cameraWriter_.round())
      .arg(cvRound(cameraWriter_.capture_duration()))
      .arg(cameraWriter_.num_saved_frames())
      .arg(write_drops)
      .arg(capture_drops));
}


void MainWindow::onCurrentImageChanged()
{
  updateMeasurements();
}



//void MainWindow::updateMeasureChannels()
//{
//  if ( is_visible(measureDisplay) && is_visible(measureDisplay->measureSelector())) {
//
//    const c_data_frame::sptr & currentDataFrame =
//        inputSourceView->currentFrame();
//
//    if ( currentDataFrame ) {
//
//      QStringList displayNames;
//
//      const c_data_frame::ImageDisplays & displays =
//          currentDataFrame->get_available_image_displays();
//
//      for ( auto ii = displays.begin(); ii != displays.end(); ++ii) {
//        displayNames.append(ii->first.c_str());
//      }
//
//
//      measureDisplay->measureSelector()->updateAvailableDataChannels(displayNames);
//    }
//  }
//}

void MainWindow::updateMeasurements()
{
  if( is_visible(profileGraph_ctl) ) {

    QImageViewer::current_image_lock lock(centralDisplay_);

    profileGraph_ctl->showProfilePlot(profileGraph_ctl->currentLine(),
        centralDisplay_->currentImage(),
        centralDisplay_->currentMask());
  }

  if( !QMeasureProvider::requested_measures().empty() && centralDisplay_->rectShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(centralDisplay_);

    QList<QMeasureProvider::MeasuredFrame> measuredFrames;
    QMeasureProvider::MeasuredFrame frame;

     const bool fOK =
         QMeasureProvider::compute(&frame,
             centralDisplay_->currentImage(),
             centralDisplay_->currentMask(),
             centralDisplay_->rectShape()->iSceneRect());

     if ( fOK ) {
       measuredFrames.append(frame);
     }

     if (!measuredFrames.empty()) {
       CF_DEBUG("FIXME: check if this measurement is handled correctly");
       Q_EMIT QMeasureProvider::instance()->framesMeasured(measuredFrames);
     }

//    QMeasureProvider::compute(centralDisplay_->currentImage(),
//        centralDisplay_->currentMask(),
//        centralDisplay_->rectShape()->iSceneRect());
  }
}

void MainWindow::onMeasureRightNowRequested()
{
  updateMeasurements();
}

void MainWindow::onCurrentDisplayImageChanged()
{
  if ( !lockDiplayImageWriter_ && diplayImageWriter_.started() ) {

    lockDiplayImageWriter_ = true;

    if ( !centralDisplay_->isVisible() ) {
      diplayImageWriter_.stop();
    }
    else if ( !centralDisplay_->displayImage().empty() ) {
      diplayImageWriter_.write(centralDisplay_->displayImage());
    }

    lockDiplayImageWriter_ = false;
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
            menuView);

    indigoFocuser_->setIndigoClient(indigoClient_);
    indigoFocuserDock_->hide();

  }
#endif // HAVE_INDIGO
}

void MainWindow::setupDisplayImageVideoWriter()
{
  mainToolbar_->addWidget(displayImageVideoWriterToolButton_ =
      createDisplayVideoWriterOptionsToolButton(&diplayImageWriter_, this));
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

void MainWindow::onExposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed)
{
  switch (status) {
    case QImagingCamera::Exposure_working:
      exposure_status_ctl->setText(QString::asprintf("E: %.1f / %.1f [s]", elapsed * 1e-3, exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_success:
      exposure_status_ctl->setText(QString::asprintf("E: %.1f [s] OK", exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_failed:
      exposure_status_ctl->setText(QString::asprintf("E: %.1f [s] FAILED", exposure * 1e-3));
      exposure_status_ctl->show();
      break;
    case QImagingCamera::Exposure_idle:
      exposure_status_ctl->setText("");
      exposure_status_ctl->hide();
      break;
  }
}




} /* namespace serimager */
