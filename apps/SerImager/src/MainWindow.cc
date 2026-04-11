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

  setCentralWidget(_liveDisplay = new QLiveDisplay(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  QFFStreams::load();
#if HAVE_QLCSCTPCamera
  QLCSCTPStreams::load();
#endif //HAVE_QLCSCTPCamera

  _liveThread = new QLivePipelineThread(this);
  _liveThread->setDisplay(_liveDisplay);


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

  QObject::connect(_liveThread, &QLivePipelineThread::pipelineChanged,
      this, &ThisClass::toggleUpdateTimer, Qt::QueuedConnection);

  QObject::connect(_liveDisplay, &QImageViewer::currentImageChanged,
      this, &ThisClass::onCurrentImageChanged);

  QObject::connect(_liveDisplay, &QImageViewer::displayImageChanged,
      this, &ThisClass::onCurrentDisplayImageChanged);

  QObject::connect(_liveDisplay, &QLiveDisplay::onMouseMove, this,
      [this](QMouseEvent * e) {
        mouse_status_ctl->setText(_liveDisplay->statusStringForPixel(e->pos()));
        mouse_status_ctl->show();
      });

  QObject::connect(_liveDisplay, &QLiveDisplay::onMouseLeaveEvent, this,
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
    if (_liveDisplay) {
      _liveDisplay->rectShape()->setSceneRect(QPointF(x,y), QPointF(x + w,y + h));
    }
  });

  set_ctlbind_get_roi_callback([this](double * x, double * y, double * w, double * h) {
    if ( _liveDisplay ) {
      const auto roi = _liveDisplay->rectShape();
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
      return _liveDisplay->isVisible() && roi->isVisible();
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
  if ( _updateTimerId >= 0 ) {
    killTimer(_updateTimerId);
  }
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

    const QString selectedPprocessor =
        settings.value(QString("imageProcessor/selected_processor")).toString();

    if ( !selectedPprocessor.isEmpty() ) {
      imageProcessor_ctl->setSelectedProcessor(selectedPprocessor);
    }

  }

  if ( dataframeProcessor_ctl ) {

    const QString selectedProcessor =
        settings.value(QString("dataframeProcessor/selected_processor")).toString();

    if ( !selectedProcessor.isEmpty() ) {
      dataframeProcessor_ctl->setSelectedProcessor(selectedProcessor);
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

  menuEdit->addAction(copyDisplayImageAction = createAction(getIcon(ICON_copy),
      "Copy display image to clipboard (Ctrl+c)",
      "Copy display image to clipboard (Ctrl+c)",
      [this]() {
        if ( _liveDisplay->rectShape()->isVisible() ) {
          _liveDisplay->copyDisplayImageROIToClipboard(_liveDisplay->rectShape()->iSceneRect());
        }
        else {
          _liveDisplay->copyDisplayImageToClipboard();
        }
      },
      new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
          _liveDisplay, nullptr, nullptr,
          Qt::WindowShortcut)));

  menuEdit->addAction(copyDisplayViewportAction = createAction(QIcon(),
      "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
      "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
      [this]() {
        if ( _liveDisplay->isVisible() ) {
          QPixmap pxmap = _liveDisplay->sceneView()->grab();
          if ( !pxmap.isNull() ) {
            QApplication::clipboard()->setPixmap(pxmap);
          }
        }
      },
      new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
          _liveDisplay, nullptr, nullptr,
          Qt::WindowShortcut)));

  ///////////////////////////////////////////////////////////////////

  _menuViewShapes =
      menuView->addMenu(getIcon(ICON_shapes),
          "Shapes");

  _menuViewShapes->addAction(_showRectShapeAction = createCheckableAction(getIcon(ICON_roi),
      "ROI Rectangle",
      "Show / Hide ROI rectangle",
      is_visible(_liveDisplay) && _liveDisplay->rectShape()->isVisible(),
      [this](bool checked) {
        _liveDisplay->rectShape()->setVisible(checked);
      }));


  _menuViewShapes->addAction(_showLineShapeAction = createCheckableAction(getIcon(ICON_line),
      "Line Shape",
      "Show / Hide Line Shape",
      is_visible(_liveDisplay) && _liveDisplay->lineShape()->isVisible(),
      [this](bool checked) {
        _liveDisplay->lineShape()->setVisible(checked);
      }));


  _menuViewShapes->addAction(_showTargetShapeAction = createCheckableAction(getIcon(ICON_target),
      "Target Shape",
      "Show / Hide Target Shape",
      is_visible(_liveDisplay) && _liveDisplay->targetShape()->isVisible(),
      [this](bool checked) {
        _liveDisplay->targetShape()->setVisible(checked);
      }));

  /////////////////////////////////////

  menuView->addAction(showMtfControlAction = createCheckableAction(getIcon(ICON_histogram),
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
          _liveDisplay->rectShape(),
          this);

  _rectShapeActionsMenu.addAction(action = createCheckableAction(QIcon(),
      "Options..",
      "Configure ROI rectangle options",
      is_visible(_rectShapeOptionsDialogBox) && _rectShapeOptionsDialogBox->isVisible(),
      [this](bool checked) {
        _rectShapeOptionsDialogBox->setVisible(checked);
        if ( _rectShapeOptionsDialogBox->isVisible() ) {
          _rectShapeOptionsDialogBox->updateControls();
        }
      }));

  QObject::connect(_rectShapeOptionsDialogBox, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      this,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          _liveDisplay->rectShape()->setVisible(true);
          _showRectShapeAction->setChecked(true);
        }
      });

  QObject::connect(_liveDisplay->rectShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  QObject::connect(_liveDisplay->rectShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);

  QObject::connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged,
      this, &ThisClass::onCentralDisplayROIShapeChanged);


  //
  // Target shape
  //

  _targetShapeOptionsDialogBox =
      new QGraphicsTargetShapeSettingsDialogBox("Target shape options",
          _liveDisplay->targetShape(),
          this);

  _targetShapeActionsMenu.addAction(action = createCheckableAction(QIcon(),
      "Options..",
      "Configure target shape options",
      is_visible(_targetShapeOptionsDialogBox),
      [this](bool checked) {
        _targetShapeOptionsDialogBox->setVisible(checked);
      }));

  QObject::connect(_targetShapeOptionsDialogBox, &QGraphicsTargetShapeSettingsDialogBox::visibilityChanged,
      this,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          _liveDisplay->targetShape()->setVisible(true);
          _showTargetShapeAction->setChecked(true);
        }
      });


  QObject::connect(_liveDisplay->targetShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayTargetShapeChanged);

  QObject::connect(_liveDisplay->targetShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayTargetShapeChanged);


  //
  // Line shape
  //
  _lineShapeOptionsDialogBox =
      new QGraphicsLineShapeSettingsDialogBox("Line shape options",
          _liveDisplay->lineShape(),
          this);

  _lineShapeActionsMenu.addAction(action = createCheckableAction(QIcon(),
      "Options..",
      "Configure line shape options",
      is_visible(_lineShapeOptionsDialogBox),
      [this](bool checked) {
        _lineShapeOptionsDialogBox->setVisible(checked);
      }));

  QObject::connect(_lineShapeOptionsDialogBox, &QGraphicsLineShapeSettingsDialogBox::visibilityChanged,
      this,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          _liveDisplay->lineShape()->setVisible(true);
          _showLineShapeAction->setChecked(visible);
        }
      });

  QObject::connect(_liveDisplay->lineShape(), &QGraphicsShape::itemChanged,
      this, &ThisClass::onCentralDisplayLineShapeChanged);

  QObject::connect(_liveDisplay->lineShape(), &QGraphicsShape::visibleChanged,
      this, &ThisClass::onCentralDisplayLineShapeChanged);

}

void MainWindow::onCentralDisplayROIShapeChanged()
{
  QGraphicsRectShape *shape = _liveDisplay->rectShape();

  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      _showRectShapeAction->setChecked(false);
    }
    else {

      const QRectF rc = shape->sceneRect();
      const QImagingCamera::sptr &currentCamera = cameraControls_ctl->selectedCamera();
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

    if ( is_visible(_rectShapeOptionsDialogBox)) {
      _rectShapeOptionsDialogBox->updateControls();
    }
  }
}

void MainWindow::onCentralDisplayLineShapeChanged()
{
  QGraphicsLineShape *shape = _liveDisplay->lineShape();
  if( shape ) {

    if( !shape->isVisible() ) {
      shape_status_ctl->hide();
      _showLineShapeAction->setChecked(false);
    }
    else {

      const QLineF line = shape->sceneLine();

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

        QImageViewer::current_image_lock lock(_liveDisplay);

        profileGraph_ctl->showProfilePlot(line,
            _liveDisplay->currentImage(),
            _liveDisplay->currentMask());
      }
    }
  }
}

void MainWindow::onCentralDisplayTargetShapeChanged()
{
  const QGraphicsTargetShape *shape = _liveDisplay->targetShape();
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

    QGraphicsLineShape *shape = _liveDisplay->lineShape();

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
  QObject::connect(_displayScaleControl, &QScaleSelectionButton::scaleChanged, this,
      [this](int currentScale) {
        _liveDisplay->setViewScale(currentScale);
      });


  ///////////////////////////////////////////////////////////////////

  _mainToolbar->addAction(_showLiveThreadSettingsAction = createCheckableAction(getIcon(ICON_bayer),
      "Bayer",
      "Configure debayer options",
      is_visible(_liveThreadSettingsDialogBox),
      this,
      &ThisClass::onShowLiveThreadSettingsActionTriggered));

  ///////////////////////////////////////////////////////////////////

  _mainToolbar->addWidget(_measureActionsToolButton = createToolButtonWithMenu(getIcon(ICON_measures),
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
  sb->addPermanentWidget(pipeline_status_ctl = new QLabel("", this));

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

  QObject::connect(cameraControls_ctl, &QImagingCameraControlsWidget::selectedCameraChanged, this,
      [this]() {

        const QImagingCamera::sptr & camera = cameraControls_ctl->selectedCamera();

        _liveThread->setCamera(camera);

        if ( camera ) {
          QObject::connect(camera.get(), &QImagingCamera::exposureStatusUpdate,
              this, &ThisClass::onExposureStatusUpdate,
              Qt::QueuedConnection);
        }
      });

  QObject::connect(&_cameraWriter, &QCameraWriter::statusUpdate,
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
  pipelineSelector_ctl->setLiveThread(_liveThread);
}

void MainWindow::onImageProcessorParameterChanged()
{
  _liveDisplay->setCurrentProcessor(imageProcessor_ctl->currentProcessor());
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
  return _liveDisplay;
}

void MainWindow::onCurrentImageChanged()
{
  updateMeasurements();
}

void MainWindow::updateMeasurements()
{
  if( is_visible(profileGraph_ctl) ) {

    QImageViewer::current_image_lock lock(_liveDisplay);

    profileGraph_ctl->showProfilePlot(profileGraph_ctl->currentLine(),
        _liveDisplay->currentImage(),
        _liveDisplay->currentMask());
  }

  if( !QMeasureProvider::requested_measures().empty() && _liveDisplay->rectShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(_liveDisplay);

    QList<QMeasureProvider::MeasuredFrame> measuredFrames;
    QMeasureProvider::MeasuredFrame frame;

     const bool fOK =
         QMeasureProvider::compute(&frame,
             _liveDisplay->currentImage(),
             _liveDisplay->currentMask(),
             _liveDisplay->rectShape()->iSceneRect());

     if ( fOK ) {
       measuredFrames.append(frame);
     }

     if (!measuredFrames.empty()) {
       CF_DEBUG("FIXME: check if this measurement is handled correctly");
       Q_EMIT QMeasureProvider::instance()->framesMeasured(measuredFrames);
     }
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

    if ( !_liveDisplay->isVisible() ) {
      _diplayImageWriter.stop();
    }
    else if ( !_liveDisplay->displayImage().empty() ) {
      _diplayImageWriter.write(_liveDisplay->displayImage());
    }

    _lockDiplayImageWriter = false;
  }
}

void MainWindow::setupIndigoFocuser()
{
#if HAVE_INDIGO
  static bool inidigo_initialized = false;
  if( !inidigo_initialized ) {

    if( !_indigoClient ) {
      _indigoClient = new QIndigoClient("SerImager", this);
    }

    /* This shall be set only before connecting */
    indigo_use_host_suffix = true;
    indigo_set_log_level(INDIGO_LOG_INFO);

    indigo_result status;

    if( (status = _indigoClient->start()) != INDIGO_OK ) {
      QMessageBox::critical(this, "ERROR",
          "_indigoClient.start() fails");
    }
    else if( (status = _indigoClient->load_driver("indigo_focuser_focusdreampro")) ) {
      QMessageBox::critical(this, "ERROR",
          "_indigoClient.loadIndigoDriver(indigo_focuser_focusdreampro) fails");
    }

    inidigo_initialized = true;

    _indigoFocuserDock =
        addCustomDock(this,
            Qt::RightDockWidgetArea,
            "indigoFocuserDock",
            "Indigo focuser",
            _indigoFocuser = new QIndigoFocuserWidget(this),
            menuView);

    _indigoFocuser->setIndigoClient(_indigoClient);
    _indigoFocuserDock->hide();

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
      _liveThreadSettingsDialogBox->setLiveThread(_liveThread);

      QObject::connect(_liveThreadSettingsDialogBox, &QLiveThreadSettingsDialogBox::visibilityChanged, this,
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


void MainWindow::toggleUpdateTimer()
{
  bool activateUpdateTimer = false;

  if ( _liveThread ) {
    // Safe, as pipeline can be changed from GUI thread only
    const auto & pipeline = _liveThread->pipeline();
    if ( pipeline ) {
      activateUpdateTimer = true;
    }
  }

  if ( activateUpdateTimer ) {
    if( _updateTimerId < 0 ) {
      _updateTimerId = startTimer(1000);
    }
  }
  else if( _updateTimerId >= 0 ) {
    killTimer(_updateTimerId);
    _updateTimerId = -1;
  }
}

void MainWindow::timerEvent(QTimerEvent * event)
{
  if( event->timerId() == _updateTimerId ) {
    onPipelineStatusUpdate();
  }
  else {
    Base::timerEvent(event);
  }
}

void MainWindow::onPipelineStatusUpdate()
{
  if ( _liveThread ) {
    // Safe, as it can be changed from GUI thread only
    const auto & pipeline = _liveThread->pipeline();
    if ( pipeline ) {
      const int accumulated_frames = pipeline->accumulated_frames();
      const int processed_frames = pipeline->processed_frames();
      pipeline_status_ctl->setText(QString("| P: %1 A: %2").arg(processed_frames).arg(accumulated_frames));
    }
  }
}


} /* namespace serimager */
