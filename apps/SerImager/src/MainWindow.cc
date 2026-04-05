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
#include <gui/widgets/QTextInfoDialogBox.h>
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

  setCentralWidget(_centralDisplay = new QLiveDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  QFFStreams::load();
#if HAVE_QLCSCTPCamera
  QLCSCTPStreams::load();
#endif //HAVE_QLCSCTPCamera

  _liveView = new QLivePipelineThread(this);
  _liveView->setDisplay(_centralDisplay);


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

  connect(_centralDisplay, &QImageViewer::currentImageChanged,
      this, &ThisClass::onCurrentImageChanged);

  connect(_centralDisplay, &QImageViewer::displayImageChanged,
      this, &ThisClass::onCurrentDisplayImageChanged);


  connect(_centralDisplay, &QLiveDisplay::onMouseMove,
      [this](QMouseEvent * e) {
        mouse_status_ctl->setText(_centralDisplay->statusStringForPixel(e->pos()));
        mouse_status_ctl->show();
      });

  connect(_centralDisplay, &QLiveDisplay::onMouseLeaveEvent,
      [this](QEvent * e) {
        if ( mouse_status_ctl->isVisible() ) {
          mouse_status_ctl->hide();
        }
      });

  set_ctlbind_show_info_text_callback([](const std::string & title, const std::string & text) {
    QTextInfoDialogBox::show(QString::fromStdString(title), QString::fromStdString(text),
        QApplication::activeWindow());
  });

  set_ctlbind_copy_to_clipboard_callback([](const std::string & text) {
    QApplication::clipboard()->setText(QString::fromStdString(text));
  });

  set_ctlbind_get_clipboard_text_callback([]() -> std::string {
    return QApplication::clipboard()->text().toStdString();
  });

  set_ctlbind_update_roi_callback([this](double x, double y, double w, double h) {
    if (_centralDisplay) {
      _centralDisplay->rectShape()->setSceneRect(QPointF(x,y), QPointF(x + w,y + h));
    }
  });

  set_ctlbind_get_roi_callback([this](double * x, double * y, double * w, double * h) {
    if ( _centralDisplay ) {
      const auto roi = _centralDisplay->rectShape();
      const auto rc = roi->sceneRect();
      if ( x ) {
        *x = rc.x();
      }
      if ( y ) {
        *y = rc.y();
      }
      if ( w ) {
        *w = rc.width();
      }
      if ( h ) {
        *h = rc.height();
      }
      return _centralDisplay->isVisible() && roi->isVisible();
    }
    return false;
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
        imageProcessor_ctl->selectedProcessor());
  }

  if ( dataframeProcessor_ctl ) {

    settings.setValue(QString("dataframeProcessor/selected_processor"),
        dataframeProcessor_ctl->selectedProcessor());
  }

  if ( cameraControls_ctl ) {
    cameraControls_ctl->saveSettings(settings, "QImagingCameraControls");
  }

  _diplayImageWriter.saveSettings(settings, "QDisplayVideoWriter");
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( imageProcessor_ctl ) {

    const QString selected_processor =
        settings.value(QString("imageProcessor/selected_processor")).toString();

    if ( !selected_processor.isEmpty() ) {
      imageProcessor_ctl->setSelectedProcessor(selected_processor);
    }

  }

  if ( dataframeProcessor_ctl ) {

    const QString selected_processor =
        settings.value(QString("dataframeProcessor/selected_processor")).toString();

    if ( !selected_processor.isEmpty() ) {
      dataframeProcessor_ctl->setSelectedProcessor(selected_processor);
    }
  }

  if ( cameraControls_ctl ) {
    cameraControls_ctl->loadSettings(settings, "QImagingCameraControls");
  }

  _diplayImageWriter.loadSettings(settings, "QDisplayVideoWriter");
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
            if ( _centralDisplay->rectShape()->isVisible() ) {
              _centralDisplay->copyDisplayImageROIToClipboard(_centralDisplay->rectShape()->iSceneRect());
            }
            else {
              _centralDisplay->copyDisplayImageToClipboard();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              _centralDisplay, nullptr, nullptr,
              Qt::WindowShortcut)));


  menuEdit->addAction(copyDisplayViewportAction =
      createAction(QIcon(),
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          [this]() {
            if ( _centralDisplay->isVisible() ) {
              QPixmap pxmap = _centralDisplay->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
              _centralDisplay, nullptr, nullptr,
              Qt::WindowShortcut)));

  ///////////////////////////////////////////////////////////////////

  _menuViewShapes =
      menuView->addMenu(getIcon(ICON_shapes),
          "Shapes");

  _menuViewShapes->addAction(_showRectShapeAction =
      createCheckableAction(getIcon(ICON_roi),
          "ROI Rectangle",
          "Show / Hide ROI rectangle",
          is_visible(_centralDisplay) && _centralDisplay->rectShape()->isVisible(),
          [this](bool checked) {
            _centralDisplay->rectShape()->setVisible(checked);
          }));


  _menuViewShapes->addAction(_showLineShapeAction =
      createCheckableAction(getIcon(ICON_line),
          "Line Shape",
          "Show / Hide Line Shape",
          is_visible(_centralDisplay) && _centralDisplay->lineShape()->isVisible(),
          [this](bool checked) {
            _centralDisplay->lineShape()->setVisible(checked);
          }));


  _menuViewShapes->addAction(_showTargetShapeAction =
      createCheckableAction(getIcon(ICON_target),
          "Target Shape",
          "Show / Hide Target Shape",
          is_visible(_centralDisplay) && _centralDisplay->targetShape()->isVisible(),
          [this](bool checked) {
            _centralDisplay->targetShape()->setVisible(checked);
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
  _rectShapeOptionsDialogBox =
      new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
          _centralDisplay->rectShape(),
          this);

  _rectShapeActionsMenu.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure ROI rectangle options",
          is_visible(_rectShapeOptionsDialogBox) && _rectShapeOptionsDialogBox->isVisible(),
          [this](bool checked) {
            _rectShapeOptionsDialogBox->setVisible(checked);
          }));

  connect(_rectShapeOptionsDialogBox, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          _centralDisplay->rectShape()->setVisible(true);
          _showRectShapeAction->setChecked(true);
        }
      });

  connect(_centralDisplay->rectShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  connect(_centralDisplay->rectShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);


  //
  // Target shape
  //

  _targetShapeOptionsDialogBox =
      new QGraphicsTargetShapeSettingsDialogBox("Target shape options",
          _centralDisplay->targetShape(),
          this);

  _targetShapeActionsMenu.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure target shape options",
          is_visible(_targetShapeOptionsDialogBox),
          [this](bool checked) {
            _targetShapeOptionsDialogBox->setVisible(checked);
          }));

  connect(_targetShapeOptionsDialogBox, &QGraphicsTargetShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          _centralDisplay->targetShape()->setVisible(true);
          _showTargetShapeAction->setChecked(true);
        }
      });


  connect(_centralDisplay->targetShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayTargetShapeChanged);

  connect(_centralDisplay->targetShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayTargetShapeChanged);


  //
  // Line shape
  //
  _lineShapeOptionsDialogBox =
      new QGraphicsLineShapeSettingsDialogBox("Line shape options",
          _centralDisplay->lineShape(),
          this);

  _lineShapeActionsMenu.addAction(
      action = createCheckableAction(QIcon(),
          "Options..",
          "Configure line shape options",
          is_visible(_lineShapeOptionsDialogBox),
          [this](bool checked) {
            _lineShapeOptionsDialogBox->setVisible(checked);
          }));

  connect(_lineShapeOptionsDialogBox, &QGraphicsLineShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {

        action->setChecked(visible);

        if ( visible ) {

          _centralDisplay->lineShape()->setVisible(true);
          _showLineShapeAction->setChecked(visible);
        }
      });


  connect(_centralDisplay->lineShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayLineShapeChanged);

  connect(_centralDisplay->lineShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayLineShapeChanged);

}

void MainWindow::onCentralDisplayROIShapeChanged()
{
  QGraphicsRectShape *shape =
      _centralDisplay->rectShape();

  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      _showRectShapeAction->setChecked(false);
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
      _showRectShapeAction->setChecked(true);

      updateMeasurements();
    }
  }

}

void MainWindow::onCentralDisplayLineShapeChanged()
{
  QGraphicsLineShape *shape =
      _centralDisplay->lineShape();

  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      _showLineShapeAction->setChecked(false);
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
      _showLineShapeAction->setChecked(true);

      if( is_visible(profileGraph_ctl) ) {

        QImageViewer::current_image_lock lock(_centralDisplay);

        profileGraph_ctl->showProfilePlot(line,
            _centralDisplay->currentImage(),
            _centralDisplay->currentMask());
      }
    }
  }
}

void MainWindow::onCentralDisplayTargetShapeChanged()
{
  const QGraphicsTargetShape *shape = _centralDisplay->targetShape();
  if( !shape->isVisible() ) {
    shape_status_ctl->hide();
    _showTargetShapeAction->setChecked(false);
  }
  else {
    const QPointF cp = shape->mapToScene(shape->center());
    shape_status_ctl->setText(qsprintf("Center: (%g %g)", cp.x(), cp.y()));
    shape_status_ctl->show();
    _showTargetShapeAction->setChecked(true);
  }
}

void MainWindow::onPlotProfileDialogBoxVisibilityChanged(bool visible)
{
  Base::onPlotProfileDialogBoxVisibilityChanged(visible);

  if( is_visible(profileGraph_ctl) ) {

    QGraphicsLineShape *shape =
        _centralDisplay->lineShape();

    if( shape && !shape->isVisible() ) {
      shape->setVisible(true);
    }
  }
}

void MainWindow::setupMainToolbar()
{
  menuBar()->setCornerWidget(_mainToolbar =
      new QToolBar(this),
      Qt::TopRightCorner);

  _mainToolbar->setContentsMargins(0, 0, 0, 0);
  _mainToolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  _mainToolbar->setIconSize(QSize(32, 18));

  ///////////////////////////////////////////////////////////////////

  _mainToolbar->addWidget(_rectShapeActionsButton =
      createToolButtonWithPopupMenu(_showRectShapeAction,
          &_rectShapeActionsMenu));

  _mainToolbar->addWidget(_lineShapeActionsButton =
      createToolButtonWithPopupMenu(_showLineShapeAction,
          &_lineShapeActionsMenu));

  _mainToolbar->addWidget(_targetShapeActionsButton =
      createToolButtonWithPopupMenu(_showTargetShapeAction,
          &_targetShapeActionsMenu));




  ///////////////////////////////////////////////////////////////////


  _mainToolbar->addAction(showImageProcessorAction);
  _mainToolbar->addAction(showMtfControlAction);

  ///////////////////////////////////////////////////////////////////

  _mainToolbar->addWidget(_displayScaleControl = new QScaleSelectionButton());
  _displayScaleControl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(_displayScaleControl, &QScaleSelectionButton::scaleChanged,
      [this](int currentScale) {
        _centralDisplay->setViewScale(currentScale);
      });


  ///////////////////////////////////////////////////////////////////

  _mainToolbar->addAction(_showLiveThreadSettingsAction =
      createCheckableAction(getIcon(ICON_bayer),
          "Bayer",
          "Configure debayer options",
          is_visible(_liveThreadSettingsDialogBox),
          this,
          &ThisClass::onShowLiveThreadSettingsActionTriggered));

  ///////////////////////////////////////////////////////////////////

  //manToolbar_->addAction(showMeasureDisplayDialogBoxAction_);

  _mainToolbar->addWidget(_measureActionsToolButton =
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
      _cameraControlsDock =
          new QImagingCameraControlsDock("Camera controls", this,
              cameraControls_ctl = new QImagingCameraControlsWidget(this)));

  _cameraControlsDock->setObjectName("imagerSettingsDock_");
  _cameraControlsDock->titleBar()->setWindowIcon(getIcon(ICON_camera));

  menuView->addAction(_showCameraControlsAction =
      _cameraControlsDock->toggleViewAction());

  _showCameraControlsAction->setIcon(getIcon(ICON_camera));

  cameraControls_ctl->setCameraWriter(&_cameraWriter);

  connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      [this]() {

        const QImagingCamera::sptr & camera =
            cameraControls_ctl->selectedCamera();

        _liveView->setCamera(camera);

        if ( camera ) {

          connect(camera.get(), &QImagingCamera::exposureStatusUpdate,
              this, &ThisClass::onExposureStatusUpdate,
              Qt::QueuedConnection);
        }
      });

  connect(&_cameraWriter, &QCameraWriter::statusUpdate,
      this, &ThisClass::onCameraWriterStatusUpdate,
      Qt::QueuedConnection);

}

void MainWindow::setupPipelines()
{
  registerPipelineClasses();

  _pipelineSelectorDock =
      addCustomDock(this,
          Qt::RightDockWidgetArea,
          "pipelineSelectorDock_",
          "Live Pipelines",
          pipelineSelector_ctl = new QLivePipelineSelectionWidget(this),
          menuView);

  _pipelineSelectorDock->titleBar()->setWindowIcon(getIcon(ICON_process));


  _pipelineSelectorDock->hide();

  _showPipelineSelectorAction = _pipelineSelectorDock->toggleViewAction();
  _showPipelineSelectorAction->setIcon(getIcon(ICON_process));
  _showPipelineSelectorAction->setToolTip("Show / Hide live pipelines");

  //pipelineCollection_.load();
  //pipelineSelector_ctl->setPipelineCollection(&pipelineCollection_);
  pipelineSelector_ctl->loadPipelines();
  pipelineSelector_ctl->setLiveThread(_liveView);
}

void MainWindow::onImageProcessorParameterChanged()
{
  _centralDisplay->setFrameProcessor(imageProcessor_ctl->currentProcessor());
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
  return _centralDisplay->mtfDisplayFunction();
}


void MainWindow::onCameraWriterStatusUpdate()
{
  const int capture_drops = _cameraWriter.camera() ?
      _cameraWriter.camera()->drops() : 0;

  const int write_drops =
      _cameraWriter.num_dropped_frames();

  capture_status_ctl->setText(QString("|R: %1 |T: %2 s |F: %3 |D: %4:%5")
      .arg(_cameraWriter.round())
      .arg(cvRound(_cameraWriter.capture_duration()))
      .arg(_cameraWriter.num_saved_frames())
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

    QImageViewer::current_image_lock lock(_centralDisplay);

    profileGraph_ctl->showProfilePlot(profileGraph_ctl->currentLine(),
        _centralDisplay->currentImage(),
        _centralDisplay->currentMask());
  }

  if( !QMeasureProvider::requested_measures().empty() && _centralDisplay->rectShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(_centralDisplay);

    QList<QMeasureProvider::MeasuredFrame> measuredFrames;
    QMeasureProvider::MeasuredFrame frame;

     const bool fOK =
         QMeasureProvider::compute(&frame,
             _centralDisplay->currentImage(),
             _centralDisplay->currentMask(),
             _centralDisplay->rectShape()->iSceneRect());

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
  if ( !_lockDiplayImageWriter && _diplayImageWriter.started() ) {

    _lockDiplayImageWriter = true;

    if ( !_centralDisplay->isVisible() ) {
      _diplayImageWriter.stop();
    }
    else if ( !_centralDisplay->displayImage().empty() ) {
      _diplayImageWriter.write(_centralDisplay->displayImage());
    }

    _lockDiplayImageWriter = false;
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
  _mainToolbar->addWidget(_displayImageVideoWriterToolButton =
      createDisplayVideoWriterOptionsToolButton(&_diplayImageWriter, this));
}


void MainWindow::onShowLiveThreadSettingsActionTriggered(bool checked)
{
  if( checked ) {
    if( !_liveThreadSettingsDialogBox ) {

      _liveThreadSettingsDialogBox = new QLiveThreadSettingsDialogBox(this);
      _liveThreadSettingsDialogBox->setLiveThread(_liveView);

      connect(_liveThreadSettingsDialogBox, &QLiveThreadSettingsDialogBox::visibilityChanged,
          [this](bool visible) {
            if ( _showLiveThreadSettingsAction ) {
              _showLiveThreadSettingsAction->setChecked(visible);
            }
          });
    }

    _liveThreadSettingsDialogBox->show();
  }
  else if( _liveThreadSettingsDialogBox ) {
    _liveThreadSettingsDialogBox->hide();
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
