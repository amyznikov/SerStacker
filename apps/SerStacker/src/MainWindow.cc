/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/AboutDialog.h>
#include <gui/widgets/QToolbarSpacer.h>
#include <gui/widgets/QMenuWidgetAction.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qimagesave/QImageSaveOptions.h>
#include <gui/qthumbnailsview/QThumbnails.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qpipeline/QPipelineThread.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/io/load_image.h>
#include <core/debug.h>

#define APP_ICON               ":/serstacker/icons/app-icon.png"
#define ICON_reload             ":/gui/icons/reload"
#define ICON_prev               ":/gui/icons/prev"
#define ICON_next               ":/gui/icons/next"
#define ICON_like               ":/gui/icons/like"
#define ICON_dislike            ":/gui/icons/dislike"
#define ICON_close              ":/gui/icons/close"
#define ICON_histogram          ":/gui/icons/histogram"
#define ICON_marker_blue        ":/gui/icons/marker-blue"
#define ICON_reference          ":/gui/icons/reference"
#define ICON_options            ":/gui/icons/options"
#define ICON_list               ":/gui/icons/list"
#define ICON_mask               ":/gui/icons/mask"
#define ICON_frame              ":/gui/icons/frame"
#define ICON_badframe           ":/gui/icons/badframe"
#define ICON_bayer              ":/gui/icons/bayer.png"

#define ICON_copy               ":/gui/icons/copy"
#define ICON_delete             ":/gui/icons/delete"
#define ICON_search             ":/gui/icons/search"

#define ICON_roi                ":/serstacker/icons/roi.png"
#define ICON_point_size         ":/serstacker/icons/degree.png"
#define ICON_brightness         ":/serstacker/icons/brightness.png"
#define ICON_bgcolor            ":/serstacker/icons/fill-color.png"

#define ICON_cloud_rotate       ":/serstacker/icons/cloud_rotate.png"
#define ICON_cloud_view_target  ":/serstacker/icons/cloud_view_target.png"
#define ICON_cloud_view_XYZ     ":/serstacker/icons/XYZ.png"
#define ICON_cloud_view_grid    ":/serstacker/icons/grid.png"


namespace serstacker {
///////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
{
  setWindowIcon(QIcon(APP_ICON));
  updateWindowTittle();
  QImageProcessorsCollection::load();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));
  centralStackedWidget->addWidget(inputSourceView = new QInputSourceView(this));
  centralStackedWidget->addWidget(pipelineOptionsView = new QPipelineOptionsView(this));
  centralStackedWidget->addWidget(pipelineProgressImageView = new QProgressImageViewer(this));

  imageView = inputSourceView->imageView();
  cloudView = inputSourceView->cloudView();
  textView = inputSourceView->textView();

  connect(centralStackedWidget, &QStackedWidget::currentChanged,
      [this]() {

        if ( is_visible(cloudViewSettingsDialogBox) ) {
          if ( ! cloudView->isVisible() ) {
            cloudViewSettingsDialogBox->hide();
          }
        }

        if ( is_visible(mtfControl) ) {
          // force update MTF histogram
          onMtfControlVisibilityChanged(true);
        }

    });


  connect(inputSourceView, &QInputSourceView::currentViewChanged,
      this, &ThisClass::onCurrentViewVisibilityChanged);

  connect(inputSourceView, &QInputSourceView::currentFrameChanged,
      this, &ThisClass::onCurrentViewFrameChanged);

  connect(imageView, &QImageEditor::currentImageChanged,
      this, &ThisClass::onImageViewCurrentImageChanged);

  connect(imageView, &QImageEditor::displayImageChanged,
      this, &ThisClass::onCurrentViewDisplayImageChanged);

  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  setupPipelines();
  setupMainMenu();
  setupLogWidget();
  setupStatusbar();
  setupMtfControls();
  setupFileSystemTreeView();
  setupThumbnailsView();
  setupStackTreeView();
  setupStackOptionsView();
  setupImageProcessingControls();
  setupDataProcessingControls();
  setupMeasures();
  setupProfileGraph();
  setupInputSequenceView();
  setupPipelineProgressView();
  setupGeoView();


  tabifyDockWidget(fileSystemTreeDock, sequencesTreeViewDock);
  //tabifyDockWidget(sequencesTreeDock, imageProcessorSelectorDock);
  tabifyDockWidget(sequencesTreeViewDock, imageProcessorDock);


  connect(QPipelineThread::instance(), &QPipelineThread::started,
      this, &ThisClass::onPipelineThreadStarted);

  connect(QPipelineThread::instance(), &QPipelineThread::finished,
      this, &ThisClass::onPipelineThreadFinished);


  restoreState();

  imageView->set_current_processor(imageProcessor_ctl->current_processor());


  set_ctrlbind_copy_to_clipboard_callback([](const std::string & text) {
    QApplication::clipboard()->setText(QString::fromStdString(text));
  });

  set_ctrlbind_get_clipboard_text_callback([]() -> std::string {
    return QApplication::clipboard()->text().toStdString();
  });

  set_ctrlbind_update_roi_callback([this](double x, double y, double w, double h) {
    if (imageView) {
      imageView->roiShape()->setSceneRect(QPointF(x,y), QPointF(x+w,y+h));
    }
  });

  //
  // Add to the end of View menu
  //
  menuView->addSeparator();
  menuView->addAction("About SerStacker...",
      [this]() {
        AboutDialog dialog("SerStacker", getPixmap(APP_ICON), this);
        dialog.exec();
      });

}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("SerStacker");
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  Base::closeEvent(event);

  if ( event->isAccepted() ) {

    if ( imageView && imageView->scene() ) {
      imageView->scene()->disconnect();
    }
  }

}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);

  if( fileSystemTreeDock ) {
    settings.setValue("fileSystemTree/absoluteFilePath",
        fileSystemTreeDock->currentAbsoluteFilePath());
  }

  if ( cloudView ) {
    cloudView->saveSettings(settings, "QPointCloudSourceView");
  }

  if ( profileGraph_ctl ) {
    profileGraph_ctl->saveParameters("profileGraph");
  }

  if ( imageProcessor_ctl ) {

    settings.setValue(QString("imageProcessor/selected_processor"),
        imageProcessor_ctl->selected_processor());
  }

  if ( dataframeProcessor_ctl ) {

    settings.setValue(QString("dataframeProcessor/selected_processor"),
        dataframeProcessor_ctl->selected_processor());
  }

  if ( geoView ) {
    geoView->saveSettings(settings, "QGeoMapView");
  }

  if ( imageView ) {
    imageView->saveSettings(settings, "QImageSourceView");
  }

  if ( _roiOptionsDialogBox ) {
    _roiOptionsDialogBox->saveSettings(settings, "QGraphicsRectShapeSettings");
  }

  _diplayImageWriter.saveSettings(settings, "QDisplayVideoWriter");

  saveShapes(settings);
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  loadHDLSensorTypeToConfigFileMapping();

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->displayPath(settings.value(
        "fileSystemTree/absoluteFilePath").toString());
  }

  if ( cloudView ) {

    cloudView->loadSettings(settings, "QPointCloudSourceView");

    std::vector<QGLView::PlanarGridOptions> & grids = cloudView->grids();
    if ( grids.empty() ) {

      grids.emplace_back();

      QGLView::PlanarGridOptions & grid =
          grids.back();

      grid.visible = false;
      grid.step = 2;
      grid.maxDistance = 600;
    }

  }

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

  if ( geoView ) {
    geoView->loadSettings(settings, "QGeoMapView");
  }

  if ( imageView ) {
    imageView->loadSettings(settings, "QImageSourceView");
  }

  _diplayImageWriter.loadSettings(settings, "QDisplayVideoWriter");

  loadShapes(settings);
}

void MainWindow::saveShapes(QSettings & settings)
{
  const QString sectionName =
      "ImageViewShapes";

  if ( imageView ) {

    settings.setValue(QString("%1/ROI/visible").arg(sectionName),
        imageView->roiShape()->isVisible());

    QGraphicsShape::save(imageView->roiShape(), settings,
        QString("%1/ROI").arg(sectionName));
  }


  const QList<QGraphicsShape *> & shapes =
      shapes_ctl->shapes();

  settings.setValue(QString("%1/numShapes").arg(sectionName),
      shapes.size());

  for( int i = 0, n = shapes.size(); i < n; ++i ) {
    QGraphicsShape::save(shapes[i], settings,
        QString("%1/shape%2").arg(sectionName).arg(i));
  }

}

void MainWindow::loadShapes(const QSettings & settings)
{
  const QString sectionName =
      "ImageViewShapes";


  if ( imageView ) {

    QGraphicsShape::load(imageView->roiShape(), settings,
        QString("%1/ROI").arg(sectionName));

    imageView->roiShape()->setVisible(settings.value(QString("%1/ROI/visible").arg(sectionName),
        imageView->roiShape()->isVisible()).toBool());

  }


  const int numShapes =
      settings.value(QString("%1/numShapes").arg(sectionName), 0).toInt();

  for ( int i = 0; i < numShapes; ++i ) {

    QGraphicsShape * shape =
        QGraphicsShape::load(settings,
            QString("%1/shape%2").arg(sectionName).arg(i));

    if ( shape ) {
      shapes_ctl->addShape(shape);
    }
  }
}

void MainWindow::setupPipelines()
{
  registerPipelineClasses();
}

void MainWindow::setupMainMenu()
{
  Base::setupMainMenu();

  //
  // File
  //

  menuFile->addAction(reloadCurrentFileAction =
      createAction(getIcon(ICON_reload),
          "Reload",
          "Reload current file from disk (Ctrl+R)",
          [this]() {
            QWidget * w = centralStackedWidget->currentWidget();
            if ( w == imageView ) {
              // imageView->openImage(imageView->currentFileName());
            }
            else if ( w == textView ) {
              textView->showTextFile(textView->currentFileName());
            }
            else if ( w == thumbnailsView ) {
              thumbnailsView->reload();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_R),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile->addAction(selectPreviousFileAction_ =
      createAction(getIcon(ICON_prev),
          "Previous (Ctrl+PgUp)",
          "Select previous file (Ctrl+PgUp)",
          [this]() {
            thumbnailsView->selectPrevIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile->addAction(selectNextFileAction =
      createAction(getIcon(ICON_next),
          "Next (Ctrl+PgDown)",
          "Select next file (Ctrl+PgDown)",
          [this]() {
            thumbnailsView->selectNextIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile->addSeparator();

  saveImageAsAction =
      menuFile->addAction("Save current image as...",
          this, &ThisClass::onSaveCurrentImageAs);

  saveImageAsAction->setEnabled(is_visible(imageView) &&
      !imageView->currentImage().empty());

  saveDisplayImageAsAction =
      menuFile->addAction("Save current display image as...",
          this, &ThisClass::onSaveCurrentDisplayImageAs);

  saveDisplayImageAsAction->setEnabled(is_visible(imageView) &&
      !imageView->displayImage().empty());

  saveImageMaskAction =
      menuFile->addAction("Save current image mask...",
          this, &ThisClass::onSaveCurrentImageMask);

  saveImageMaskAction->setEnabled(is_visible(imageView) &&
      !imageView->currentMask().empty());

  loadImageMaskAction =
      menuFile->addAction("Set current image mask from file...",
          this, &ThisClass::onLoadCurrentImageMask);

  loadImageMaskAction->setEnabled(is_visible(imageView) &&
      !imageView->currentImage().empty());

  menuFile->addSeparator();

  loadStackAction =
      menuFile->addAction("Load stack config...",
          this, &ThisClass::onLoadStackConfig);


  menuFile->addSeparator();

  onLoadGpsTrackAction =
      menuFile->addAction("Load GPX Track...",
          this, &ThisClass::onLoadGpxTrack);


  menuFile->addSeparator();


  quitAppAction =
      menuFile->addAction("Quit",
          this, &ThisClass::close);


  //
  // Edit
  //

  menuEdit->addAction(copyDisplayImageAction =
      createAction(QIcon(),
          "Copy display image to clipboard (Ctrl+c)",
          "Copy display image to clipboard (Ctrl+c)",
          [this]() {

            if ( is_visible(cloudView) ) {
              cloudView->copyViewportToClipboard();
            }
            else if ( is_visible(imageView) ) {
              if ( imageView->roiShape()->isVisible() ) {
                imageView->copyDisplayImageROIToClipboard(imageView->roiShape()->iSceneRect());
              }
              else {
                imageView->copyDisplayImageToClipboard();
              }
            }
            else if ( is_visible(textView) ) {
              textView->copySelectionToClipboard();
            }
            else if ( is_visible(pipelineProgressImageView) ) {
              pipelineProgressImageView->copyDisplayImageToClipboard();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              inputSourceView->stackWidget(), nullptr, nullptr,
              Qt::WidgetWithChildrenShortcut)));




  menuEdit->addAction(copyDisplayViewportAction =
      createAction(QIcon(),
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          [this]() {
            if ( is_visible(cloudView) ) {
              cloudView->copyViewportToClipboard();
            }
            else if ( is_visible(imageView) ) {
              const QPixmap pxmap = imageView->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
            else if ( is_visible(pipelineProgressImageView) ) {
              const QPixmap pxmap = pipelineProgressImageView->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
              inputSourceView->stackWidget(), nullptr, nullptr,
              Qt::WidgetWithChildrenShortcut)));


  //
  // View
  //
  menuView->addAction(viewInputOptionsAction =
      createCheckableAction(QIcon(ICON_bayer),
          "Input Options...",
          "Configure input options",
          is_visible(inputOptionsDlgBox),
          this,
          &ThisClass::onViewInputOptions));

  pipelineProgressView = new QPipelineProgressView(this);
  menuBar()->setCornerWidget(pipelineProgressView, Qt::TopRightCorner );
  pipelineProgressView->hide();

  connect(pipelineProgressView, &QPipelineProgressView::progressTextChanged,
      this, &ThisClass::onStackProgressViewTextChanged,
      Qt::QueuedConnection);
}

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

  sb->addWidget(statusbarShapesLabel_ctl = new QLabel(this));
  sb->addWidget(statusbarMousePosLabel_ctl = new QLabel(this));

  sb->addPermanentWidget(statusbarShowLog_ctl = new QToolButton());
  statusbarShowLog_ctl->setDefaultAction(showLogWidgetAction);

  statusbarShapesLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);
  statusbarMousePosLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);
}

void MainWindow::setupGeoView()
{
  geoViewDock =
      addGeoMapViewDock(this,
          Qt::BottomDockWidgetArea,
          "geoViewDock",
          "GeoView",
          menuView);

  geoView =
      geoViewDock->geoView();

  geoViewDock->hide();

  connect(geoView, &QGeoMapView::openVideoFileRequested,
      this, &ThisClass::onOpenVideoFileRequested);

  connect(geoViewDock, &QGeoMapViewDock::visibilityChanged,
      [this](bool visible) {

        if ( visible ) {
          geoView->setCurrentVideoScrollpos(inputSourceView->currentFileName(),
              inputSourceView->currentScrollpos());

          static bool firstShow = true;
          if ( firstShow && !geoView->view()->visibleRegion().isEmpty() ) {
            firstShow = false;
            CF_DEBUG("C geoView->flyToPosition");
            geoView->flyToPosition(10, 10, 90, 120);
            CF_DEBUG("R geoView->flyToPosition");
          }


        }
      });

}

void MainWindow::onOpenVideoFileRequested(const QString & filename, int scrollToIndex)
{
  QWaitCursor wait(this);

  if ( pipelineProgressView ) {
    pipelineProgressView->setImageViewer(nullptr);
  }

  centralStackedWidget->setCurrentWidget(inputSourceView);

  if( filename == inputSourceView->currentFileName() || inputSourceView->openSource(filename) ) {
    if( scrollToIndex >= 0 && inputSourceView->currentScrollpos() != scrollToIndex ) {
      inputSourceView->scrollToFrame(scrollToIndex);
    }
  }

}

void MainWindow::onStackProgressViewTextChanged()
{
  // FIXME : this is ugly temporary hotfix

  const QSize hint = pipelineProgressView->sizeHint();
  const QSize size = pipelineProgressView->size();

  if( size != hint ) {
    menuBar()->adjustSize();
  }
}

void MainWindow::setupFileSystemTreeView()
{
  fileSystemTreeDock =
      addFileSystemTreeDock(this, Qt::LeftDockWidgetArea,
          "fileSystemTreeDock",
          "Directory Tree",
          menuView);

  fileSystemTreeDock->raise();

  connect(fileSystemTreeDock, &QFileSystemTreeDock::currentDirectoryChanged,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }

        centralStackedWidget->setCurrentWidget(thumbnailsView);
        thumbnailsView->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::directoryItemPressed,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }

        if ( centralStackedWidget->currentWidget() != thumbnailsView ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
          if ( thumbnailsView->currentPath() != abspath ) {
            thumbnailsView->displayPath(abspath);
          }
        }
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::customContextMenuRequested,
      this, &ThisClass::onFileSystemTreeCustomContextMenuRequested);

}

void MainWindow::setupThumbnailsView()
{

  connect(thumbnailsView, &QThumbnailsView::showInDirTreeRequested,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }

        if ( fileSystemTreeDock ) {
          fileSystemTreeDock->show(),
          fileSystemTreeDock->raise(),
          fileSystemTreeDock->displayPath(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::currentIconChanged,
      [this](const QString & abspath) {
        if ( !is_visible(thumbnailsView) ) {
          openImage(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::iconDoubleClicked,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::iconEnterPressed,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::customContextMenuRequested,
      this, &ThisClass::onThumbnailsViewCustomContextMenuRequested);

}

void MainWindow::setupStackTreeView()
{
  sequencesTreeViewDock =
      addInputSequenceTreeDock(this,
          Qt::LeftDockWidgetArea,
          "sequencesTreeDock",
          "Sequences",
          menuView);

  sequencesTreeView =
      sequencesTreeViewDock->treeView();

  sequencesTreeView->loadSequences();

  connect(sequencesTreeView, &QInputSequencesTree::currentItemChanged,
      this, &ThisClass::onStackTreeCurrentItemChanged);

  connect(sequencesTreeView, &QInputSequencesTree::itemDoubleClicked,
      this, &ThisClass::onStackTreeItemDoubleClicked);

  connect(sequencesTreeView, &QInputSequencesTree::showImageSequenceOptionsClicked,
      this, &ThisClass::onShowImageSequenceOptions);

  connect(sequencesTreeView, &QInputSequencesTree::imageSequenceCollectionChanged,
        this, &ThisClass::saveCurrentWork );

  connect(sequencesTreeView, &QInputSequencesTree::imageSequenceSourcesChanged,
      [this](const c_image_sequence::sptr & sequence) {
        if ( pipelineOptionsView->isVisible() && pipelineOptionsView->current_sequence() == sequence ) {
          // fixme: temporary hack to force update options views
          pipelineOptionsView->set_current_sequence(sequence);
        }
        saveCurrentWork();
  });

  connect(sequencesTreeView, &QInputSequencesTree::imageSequenceNameChanged,
      this, &ThisClass::saveCurrentWork);

}

void MainWindow::setupStackOptionsView()
{
  connect(pipelineOptionsView, &QPipelineOptionsView::closeWindowRequested,
      [this]() {
        if ( !QPipelineThread::isRunning() ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
        }
        else {
          centralStackedWidget->setCurrentWidget(imageView);
          pipelineProgressView->setImageViewer(imageView);
        }
      });

  connect(pipelineOptionsView, &QPipelineOptionsView::parameterChanged,
      this, &ThisClass::saveCurrentWork);

  connect(pipelineOptionsView, &QPipelineOptionsView::cloneCurrentPipelineRequested,
      [this]() {

        QWaitCursor wait (this);

        std::vector<c_image_sequence::sptr> selectedSequences;

        sequencesTreeView->getSelectedSequences(&selectedSequences);

        if ( !selectedSequences.empty() ) {
          if ( pipelineOptionsView->cloneCurrentPipeline(selectedSequences) ) {
            saveCurrentWork();
          }
        }
      });

}

void MainWindow::checkIfBadFrameSelected()
{

  bool isBadFrameSelected = false;

  if( is_visible(inputSourceView) ) {

    const c_input_source::sptr & currentSource =
        inputSourceView->currentSource();

    if( currentSource ) {
      isBadFrameSelected =
          currentSource->is_badframe(std::max(0,
              currentSource->curpos() - 1));
    }

    if( badframeAction->isChecked() != isBadFrameSelected ) {
      badframeAction->setChecked(isBadFrameSelected);
    }
  }
}

void MainWindow::onWriteDisplayVideo()
{
  // In rare cases for some reason this call may become recursive

  if( _lockDiplayImageWriter || !_diplayImageWriter.started() || _diplayImageWriter.paused() ) {
    return;
  }

  _lockDiplayImageWriter = true;

  QWidget * currentView =
      inputSourceView->currentView();

  if( currentView == imageView ) {

    if( !_diplayImageWriter.writeViewPort() ) {
      if( !imageView->displayImage().empty() ) {
        _diplayImageWriter.write(imageView->displayImage());
      }
    }
    else {

      QImage image =
          imageView->grabViewportPixmap().toImage().convertToFormat(
              QImage::Format_BGR888);

      if( !image.isNull() ) {

        _diplayImageWriter.write(cv::Mat(image.height(), image.width(), CV_8UC3,
            (void*) image.constBits(),
            image.bytesPerLine()));
      }

    }
  }

  else if( currentView == cloudView ) {

    QImage image =
        cloudView->grabFramebuffer();

    if( !image.isNull() ) {

      if ( image.format() != QImage::Format_BGR888 ) {
        // In some versions of Qt the direct conversion of ARGB32_Premultiplied to BGR888 may work incorrectly
        image.convertTo(QImage::Format_RGBA8888);
        image.convertTo(QImage::Format_BGR888);
      }

      _diplayImageWriter.write(cv::Mat(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine()));
    }
  }

  _lockDiplayImageWriter = false;
}


void MainWindow::onLoadGpxTrack()
{
  if ( !geoView->isVisible() ) {
    geoViewDock->show();
    geoViewDock->raise();
  }

  geoView->importGpxTrack();
}


void MainWindow::onCurrentViewVisibilityChanged()
{
  if ( !is_visible(cloudView)  ) {

    if( is_visible(cloudViewSettingsDialogBox) ) {
      cloudViewSettingsDialogBox->hide();
    }

    if( is_visible(_glGridSettingsDialog) ) {
      _glGridSettingsDialog->setOptions(nullptr);
      _glGridSettingsDialog->hide();
    }
  }

  if( is_visible(mtfControl) ) { // force update MTF histogram
    onMtfControlVisibilityChanged(true);
  }

  checkIfBadFrameSelected();


  QWidget * currentView =
      inputSourceView->currentView();

  if( currentView != textView ) {
    toggleFindTextDialogBox(false);
  }

  if ( currentView == imageView ) {

    if ( !currentView->isVisible() ) {
      saveImageAsAction->setEnabled(false);
      saveDisplayImageAsAction->setEnabled(false);
      copyDisplayImageAction->setEnabled(false);
      copyDisplayViewportAction->setEnabled(false);
      saveImageMaskAction->setEnabled(false);
      loadImageMaskAction->setEnabled(false);
      _displayImageVideoWriterToolButton->setEnabled(false);
    }
    else {
      const bool hasimage =
          !imageView->currentImage().empty();

      const bool hasmask =
          !imageView->currentMask().empty();

      const bool hasdisplayimage =
          !imageView->displayImage().empty();

      saveImageAsAction->setEnabled(hasimage);
      saveDisplayImageAsAction->setEnabled(hasdisplayimage);
      copyDisplayImageAction->setEnabled(hasdisplayimage);
      copyDisplayViewportAction->setEnabled(true);
      saveImageMaskAction->setEnabled(hasmask);
      loadImageMaskAction->setEnabled(hasimage);
      _displayImageVideoWriterToolButton->setEnabled(true);
    }

  }
  else  if ( currentView == cloudView ) {

    saveImageAsAction->setEnabled(false);
    saveImageMaskAction->setEnabled(false);
    loadImageMaskAction->setEnabled(false);

    if ( !currentView->isVisible() ) {
      saveDisplayImageAsAction->setEnabled(false);
      copyDisplayImageAction->setEnabled(false);
      copyDisplayViewportAction->setEnabled(false);
      _displayImageVideoWriterToolButton->setEnabled(false);
    }
    else {
      saveDisplayImageAsAction->setEnabled(true);
      copyDisplayImageAction->setEnabled(true);
      copyDisplayViewportAction->setEnabled(true);
      _displayImageVideoWriterToolButton->setEnabled(true);
    }
  }
  else {

    saveImageAsAction->setEnabled(false);
    saveDisplayImageAsAction->setEnabled(false);
    copyDisplayImageAction->setEnabled(false);
    copyDisplayViewportAction->setEnabled(false);
    saveImageMaskAction->setEnabled(false);
    loadImageMaskAction->setEnabled(false);
    _displayImageVideoWriterToolButton->setEnabled(false);
  }

}


void MainWindow::onCurrentViewFrameChanged()
{
  checkIfBadFrameSelected();

  if( is_visible(geoView) && is_visible(inputSourceView) ) {

    geoView->setCurrentVideoScrollpos(inputSourceView->currentFileName(),
        inputSourceView->currentScrollpos());
  }
}

void MainWindow::onCurrentViewDisplayImageChanged()
{
  QWidget * currentView =
      inputSourceView->currentView();

  if ( currentView == imageView ) {

    const bool isvisible =
        imageView->isVisible();

    const bool hasdisplayimage =
        !imageView->displayImage().empty();

    saveDisplayImageAsAction->setEnabled(isvisible && hasdisplayimage);
    copyDisplayImageAction->setEnabled(isvisible && hasdisplayimage);
    copyDisplayViewportAction->setEnabled(isvisible);

    if ( _diplayImageWriter.started() ) {

      if ( !isvisible ) {
        _diplayImageWriter.stop();
      }
      else {
        onWriteDisplayVideo();
      }
    }
  }
  else if ( currentView == cloudView ) {

    const bool isvisible =
        cloudView->isVisible();

    saveDisplayImageAsAction->setEnabled(isvisible);
    copyDisplayImageAction->setEnabled(isvisible);
    copyDisplayViewportAction->setEnabled(isvisible);

    if( _diplayImageWriter.started() ) {

      if( !isvisible ) {
        _diplayImageWriter.stop();
      }
      else {
        onWriteDisplayVideo();
      }
    }
  }
  else {
    saveDisplayImageAsAction->setEnabled(false);
    copyDisplayImageAction->setEnabled(false);
    copyDisplayViewportAction->setEnabled(false);
  }

}

void MainWindow::onImageViewCurrentImageChanged()
{
  const bool isvisible =
      imageView->isVisible();

  const bool hasimage =
      !imageView->currentImage().empty();

  const bool hasmask =
      !imageView->currentMask().empty();

  saveImageAsAction->setEnabled(isvisible && hasimage);
  saveImageMaskAction->setEnabled(isvisible && hasmask);
  loadImageMaskAction->setEnabled(isvisible && hasimage);

  if( isvisible ) {

    imageSizeLabel_ctl->setText(qsprintf("%dx%d",
        imageView->currentImage().cols,
        imageView->currentImage().rows));

    updateProfileGraph();

    updateMeasurements();

    // checkIfBadFrameSelected();
  }

  if( is_visible(mtfControl) ) {
    onMtfControlVisibilityChanged(true);
  }

}


void MainWindow::onImageProcessorParameterChanged()
{
  Base::onImageProcessorParameterChanged();

  if( imageProcessor_ctl ) {
    if( imageView ) {
      imageView->set_current_processor(imageProcessor_ctl->current_processor());
    }
    if( pipelineProgressImageView ) {
      pipelineProgressImageView->set_current_processor(imageProcessor_ctl->current_processor());
    }
  }
}

void MainWindow::onDataframeProcessorParameterChanged()
{
  // Base::onDataframeProcessorParameterChanged();
  if ( dataframeProcessor_ctl ) {
    inputSourceView->setCurrentProcessor(dataframeProcessor_ctl->currentProcessor());
  }
}

IMtfDisplay * MainWindow::getCurrentMtfDisplay()
{
  if ( is_visible(inputSourceView) ) {
    return inputSourceView->mtfDisplay();
  }

  if ( is_visible(pipelineProgressImageView) ) {
    return pipelineProgressImageView->mtfDisplay();
  }

  return nullptr;
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  IMtfDisplay * currentMtfDisplay =
      getCurrentMtfDisplay();

  mtfControl->setMtfDisplaySettings(currentMtfDisplay);

  const QString currentFileName =
      mtfControl->mtfDisplaySettings() ?
          QFileInfo(inputSourceView->currentFileName()).fileName() :
          "";

  if( currentFileName.isEmpty() ) {
    mtfControl->setWindowTitle("Adjust Display Levels ...");
  }
  else {
    mtfControl->setWindowTitle(qsprintf("Adjust Display Levels: %s",
        currentFileName.toUtf8().constData()));
  }

}

void MainWindow::onShowCloudViewSettingsDialogBoxActionClicked(bool checked)
{
  if ( checked && !cloudViewSettingsDialogBox ) {

    cloudViewSettingsDialogBox = new QPointCloudViewSettingsDialogBox(this);
    cloudViewSettingsDialogBox->setCloudView(cloudView);
    connect(cloudViewSettingsDialogBox, &QPointCloudViewSettingsDialogBox::visibilityChanged,
        showCloudViewSettingsDialogBoxAction, &QAction::setChecked);
  }

  if ( cloudViewSettingsDialogBox ) {
    if ( !checked ) {
      cloudViewSettingsDialogBox->hide();
    }
    else {
      cloudViewSettingsDialogBox->setWindowTitle(QFileInfo(inputSourceView->currentFileName()).fileName());
      cloudViewSettingsDialogBox->showNormal();
    }
  }
}

void MainWindow::onShowCloudSettingsDialogBoxActionClicked(bool checked)
{
  if ( checked && !cloudSettingsDialogBox ) {

    cloudSettingsDialogBox = new QGlPointCloudSettingsDialogBox(this);
    // cloudSettingsDialogBox->setCloudView(cloudView);
    connect(cloudSettingsDialogBox, &QGlPointCloudSettingsDialogBox::visibilityChanged,
        showCloudSettingsDialogBoxAction, &QAction::setChecked);
  }

  if ( cloudSettingsDialogBox ) {
    if ( !checked ) {
      cloudSettingsDialogBox->hide();
    }
    else {
      cloudSettingsDialogBox->setWindowTitle(QFileInfo(inputSourceView->currentFileName()).fileName());
      cloudSettingsDialogBox->showNormal();
    }
  }
}

void MainWindow::onUpdateAvailableMeasureDataChannelsRequired()
{
  updateMeasureChannels();
}

void MainWindow::updateMeasureChannels()
{
  if ( is_visible(measureDisplay) && is_visible(measureDisplay->measureSelector())) {

    const c_data_frame::sptr & currentDataFrame =
        inputSourceView->currentFrame();

    if ( currentDataFrame ) {

      QStringList displayNames;

      const c_data_frame::ImageDisplays & displays =
          currentDataFrame->get_available_image_displays();

      for ( auto ii = displays.begin(); ii != displays.end(); ++ii) {
        displayNames.append(ii->first.c_str());
      }


      measureDisplay->measureSelector()->updateAvailableDataChannels(displayNames);
    }
  }
}

void MainWindow::updateMeasurements()
{
  updateMeasureChannels();

  if( !QMeasureProvider::requested_measures().empty() && is_visible(imageView) && imageView->roiShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(imageView);

    if ( !imageView->currentImage().empty() ) {

      QList<QMeasureProvider::MeasuredFrame> measuredFrames;

      if (true) {
        QMeasureProvider::MeasuredFrame frame;

        const bool fOK =
            QMeasureProvider::compute(&frame,
                imageView->currentImage(),
                imageView->currentMask(),
                imageView->roiShape()->iSceneRect());

        if ( fOK ) {
          measuredFrames.append(std::move(frame));
        }
      }

      const QStringList & requestedChannels =
          QMeasureProvider::requested_channels();

      if ( !requestedChannels.empty() ) {

        const c_data_frame::sptr & currentDataFrame =
            inputSourceView->currentFrame();

        if ( currentDataFrame ) {

          cv::Mat image, mask, data;

          for ( const QString & displayName : requestedChannels ) {
            if ( currentDataFrame->get_image(displayName.toStdString(), image, mask, data) ) {

              QMeasureProvider::MeasuredFrame frame;

              const bool fOK = QMeasureProvider::compute(&frame, image, mask,
                  imageView->roiShape()->iSceneRect());

              if ( fOK ) {
                frame.dataChannel = displayName;
                measuredFrames.append(std::move(frame));
              }
            }
          }
        }
      }

      if (!measuredFrames.empty()) {
        Q_EMIT QMeasureProvider::instance()->framesMeasured(measuredFrames);
      }
    }
  }
}

void MainWindow::updateProfileGraph(QGraphicsItem * lineItem)
{
  if( is_visible(profileGraph_ctl) && is_visible(imageView) ) {

    if( QGraphicsLineShape *lineShape = dynamic_cast<QGraphicsLineShape*>(lineItem) ) {

      profileGraph_ctl->showProfilePlot(lineShape->sceneLine(),
          imageView->currentImage(),
          imageView->currentMask());

    }
    else {

      QLine line =
          profileGraph_ctl->currentLine();

      if ( line.isNull() && is_visible(imageView) ) {

        const QList<QGraphicsItem *> items =
            imageView->scene()->items();

        for ( const QGraphicsItem * item : items ) {
          if( const QGraphicsLineShape *lineShape = dynamic_cast<const QGraphicsLineShape*>(item) ) {
            line = lineShape->sceneLine().toLine();
            break;
          }
        }
      }

      profileGraph_ctl->showProfilePlot(line,
          imageView->currentImage(),
          imageView->currentMask());

    }
  }
}

void MainWindow::onShowProfileGraphActionTriggered(bool checked)
{
  bool profileGraphAlreadyCreated =
      profileGraph_ctl != nullptr;

  Base::onShowProfileGraphActionTriggered(checked);

  if ( !profileGraphAlreadyCreated && profileGraph_ctl != nullptr ) {
    profileGraph_ctl->loadParameters("profileGraph");
  }
}

void MainWindow::onMeasureRightNowRequested()
{
  updateMeasurements();
}

void MainWindow::showImageViewOptions(bool show)
{
  if( !show ) {
    if( imageViewOptionsDlgBox ) {
      delete imageViewOptionsDlgBox;
      imageViewOptionsDlgBox = nullptr;
    }
  }
  else {
    if( !imageViewOptionsDlgBox ) {
      imageViewOptionsDlgBox = new QImageViewOptionsDlgBox(this);
      imageViewOptionsDlgBox->setImageViewer(imageView);

      connect(imageViewOptionsDlgBox, &QImageViewOptionsDlgBox::visibilityChanged,
          [this](bool visible) {
            if ( editMaskAction->isChecked() != visible ) {
              editMaskAction->setChecked(visible);
            }
          });

      connect(imageViewOptionsDlgBox, &QImageViewOptionsDlgBox::finished,
          [this](int) {
            delete imageViewOptionsDlgBox;
            imageViewOptionsDlgBox = nullptr;
          });
    }
    imageViewOptionsDlgBox->show();
  }
}


void MainWindow::openImage(const QString & abspath)
{
  QWaitCursor wait(this);

  if ( pipelineProgressView ) {
    pipelineProgressView->setImageViewer(nullptr);
  }

  centralStackedWidget->setCurrentWidget(inputSourceView);
  inputSourceView->openSource(abspath);
}



void MainWindow::onFileSystemTreeCustomContextMenuRequested(const QPoint & pos,
    const QFileInfoList & selectedItems )
{
  QMenu menu;

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->fillContextMenu(menu, selectedItems);
  }

  if ( !menu.isEmpty() ) {
    menu.exec(pos);
  }
}



void MainWindow::onThumbnailsViewCustomContextMenuRequested(const QPoint & pos)
{
  QMenu poupupMenu;
  thumbnailsView->populateContextMenu(&poupupMenu, pos);
  if( !poupupMenu.isEmpty() ) {
    poupupMenu.exec(thumbnailsView->contextMenuPosToGlobal(pos));
  }
}

void MainWindow::onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source)
{
  if ( source ) {
    openImage(source->cfilename());
  }
  else if ( sequence ) {
    if ( centralStackedWidget->currentWidget() == thumbnailsView ) {
      thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
    }
    else {
      pipelineOptionsView->set_current_sequence(sequence);
      centralStackedWidget->setCurrentWidget(pipelineOptionsView);
    }
  }

}

void MainWindow::onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source)
{
  QWidget * currentCentralWidget =
      centralStackedWidget->currentWidget();

  if ( !QPipelineThread::isRunning() ) {

    if ( source ) {

      if( currentCentralWidget != inputSourceView ) {
        openImage(source->cfilename());
      }
      else {
        thumbnailsView->setCurrentPath(QFileInfo(source->cfilename()).absolutePath(), false);
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }
    }

    else if ( sequence ) {

      if( currentCentralWidget == pipelineOptionsView ) {
        thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }
      else {
        pipelineOptionsView->set_current_sequence(sequence);
        centralStackedWidget->setCurrentWidget(pipelineOptionsView);
      }
    }

  }
  else  {

    if ( source ) {

      pipelineProgressView->setImageViewer(nullptr);

      if ( currentCentralWidget != inputSourceView ) {
        openImage(source->cfilename());
      }
      else {
        thumbnailsView->setCurrentPath(QFileInfo(source->cfilename()).absolutePath(), false);
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }

    }
    else if( sequence ) {

      if( currentCentralWidget == pipelineProgressImageView ) {
        pipelineProgressView->setImageViewer(nullptr);
        pipelineOptionsView->set_current_sequence(sequence);
        centralStackedWidget->setCurrentWidget(pipelineOptionsView);
      }

      else if( currentCentralWidget == pipelineOptionsView ) {
        thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }

      else {
        pipelineProgressView->setImageViewer(pipelineProgressImageView);
        centralStackedWidget->setCurrentWidget(pipelineProgressImageView);
      }
    }
  }

}


void MainWindow::onShowImageSequenceOptions(const c_image_sequence::sptr & sequence)
{
  if ( sequence ) {

    if ( pipelineProgressView ) {
      pipelineProgressView->setImageViewer(nullptr);
    }

    pipelineOptionsView->set_current_sequence(sequence);
    centralStackedWidget->setCurrentWidget(pipelineOptionsView);
  }
}

void MainWindow::onPipelineThreadStarted()
{
  if ( centralStackedWidget->currentWidget() != pipelineProgressImageView ) {
    centralStackedWidget->setCurrentWidget(pipelineProgressImageView);
  }

  pipelineProgressImageView->set_current_processor(imageProcessor_ctl->current_processor());
  pipelineProgressView->setImageViewer(pipelineProgressImageView);

  if ( !pipelineProgressView->isVisible() ) {
    pipelineProgressView->show();
  }
}

void MainWindow::onPipelineThreadFinished()
{
  if( pipelineProgressView ) {

    // it may be that there is next task in queue,
    // don't blink with this dialog box
    QTimer::singleShot(1000,
        [this]() {

          if ( !QPipelineThread::isRunning() ) {
            if ( pipelineProgressView->isVisible() ) {
              pipelineProgressView->hide();
            }
            if ( pipelineOptionsView->isVisible() ) {
              pipelineOptionsView->setEnabled(true);
            }
          }
        });
  }
}

void MainWindow::onSaveCurrentImageAs()
{
  if( is_visible(imageView) ) {
    if( !imageView->currentImage().empty() ) {

      const QString savedFileName =
          saveImageFileAs(this,
              imageView->currentImage(),
              imageView->currentMask(),
              imageView->current_processor(),
              inputSourceView->currentFileName());

      if( !savedFileName.isEmpty() ) {
        CF_WARNING("FIXME: Set CurrentFileName correctly !");
        imageView->setCurrentFileName(savedFileName);
      }
    }
  }
  else if( is_visible(cloudView) ) {

    QImage image =
        cloudView->grab().toImage().convertToFormat(
            QImage::Format_BGR888);

    if( !image.isNull() ) {

      const cv::Mat M(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine());

      saveImageFileAs(this,
          M,
          cv::Mat(),
          nullptr,
          inputSourceView->currentFileName());
    }
  }

}

void MainWindow::onSaveCurrentDisplayImageAs()
{
  if( is_visible(imageView) ) {

    const cv::Mat &displayImage =
        imageView->displayImage();

    if( !displayImage.empty() ) {

      saveImageFileAs(this,
          displayImage,
          cv::Mat(),
          nullptr,
          inputSourceView->currentFileName());
    }
  }
  else if( is_visible(cloudView) ) {

    QImage image =
        cloudView->grab().toImage().convertToFormat(
            QImage::Format_BGR888);

    if( !image.isNull() ) {

      const cv::Mat displayImage(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine());

      saveImageFileAs(this,
          displayImage,
          cv::Mat(),
          nullptr,
          inputSourceView->currentFileName());
    }
  }
}

void MainWindow::onSaveCurrentImageMask()
{
  if( is_visible(imageView) ) {

    const cv::Mat &currentMask =
        imageView->currentMask();

    if( !currentMask.empty() ) {
      saveImageFileAs(this,
          imageView->currentMask(),
          cv::Mat(),
          nullptr,
          qsprintf("%s.mask.%dx%d.png",
              inputSourceView->currentFileName().toUtf8().constData(),
              currentMask.cols, currentMask.rows));
    }
  }
}

void MainWindow::onLoadCurrentImageMask()
{
  if( is_visible(imageView) ) {

    const cv::Mat &currentImage =
        imageView->currentImage();

    if( !currentImage.empty() ) {

      static const QString keyName =
          "lastImageMask";

      QSettings settings;

      static const QString filter =
          "Image files *.tiff *.tif *.png *.jpg (*.tiff *.tif *.png *.jpg);;\n"
          "All files (*);;";

      QString fileName =
          QFileDialog::getOpenFileName(this,
              "Select binary image", settings.value(keyName).toString(),
              filter,
              nullptr);

      if( fileName.isEmpty() ) {
        return;
      }

      settings.setValue(keyName, fileName);

      cv::Mat image;

      if( !load_image(fileName.toStdString(), image) ) {
        QMessageBox::critical(this, "Error",
            "load_image() fails.\n"
                "Can not load image from specified file");
        return;
      }

      if( image.type() != CV_8UC1 || image.size() != currentImage.size() ) {
        QMessageBox::critical(this, "Error",
            QString("Not appropriate mask image: %1x%2 depth=%3 channels=%4.\n"
                "Must be CV_8UC1 of the same size as current image (%5x%6)")
                .arg(image.cols)
                .arg(image.rows)
                .arg(image.depth())
                .arg(image.channels())
                .arg(currentImage.cols)
                .arg(currentImage.rows));
        return;
      }

      imageView->setMask(image, false);
    }
  }
}


void MainWindow::saveCurrentWork()
{
  QWaitCursor wait (this);

  if( !pipelineOptionsView->current_sequence() ) {
    pipelineOptionsView->set_current_sequence(nullptr);
  }

  sequencesTreeView->saveSequences();
}

void MainWindow::onLoadStackConfig()
{

#if 0
  static const QString loadStackConfigSavedPathKeyName =
      "loadStackConfigSavedPath";

  QSettings settings;

  QString savedPathFileName =
      settings.value(loadStackConfigSavedPathKeyName).toString();

  const QString filter =
      "Config files (*.cfg) ;;"
      "All files (*.*)";

  QStringList selectedFileNames =
      QFileDialog::getOpenFileNames(this,
          "Select stack config files",
          savedPathFileName,
          filter,
          nullptr,
          QFileDialog::ReadOnly);


  if ( selectedFileNames.isEmpty() ) {
    return;
  }

  settings.setValue(loadStackConfigSavedPathKeyName,
      selectedFileNames[0]);

  bool hasChanges = false;

  for ( int i = 0, n = selectedFileNames.size(); i < n; ++i ) {

    c_image_sequence::sptr sequence =
        c_image_sequence::load(selectedFileNames[i].toStdString());

    if ( !sequence ) {

      if ( i == n - 1 ) {
        QMessageBox::critical(this,
            "ERROR",
            QString("Can not load %1.\nSee error log for details.").arg(selectedFileNames[i]));
        break;
      }

      const int responce =
          QMessageBox::critical(this, "ERROR",
              QString("Can not load %1.\n"
                  "See error log for details.\n"
                  "Continue loading ?").arg(selectedFileNames[i]),
              QMessageBox::Yes | QMessageBox::No);

      if ( responce != QMessageBox::Yes ) {
        break;
      }

      continue;
    }


    int pos = image_sequences_->indexof(sequence->name());
    if ( pos < 0 ) {
      image_sequences_->add(sequence);
      hasChanges = true;
    }
    else {

      const int responce =
          QMessageBox::critical(this, "ERROR",
              QString("Stack with name '%1' already exists.\n"
                  "Replace existing ?").arg(QString(sequence->cname())),
              QMessageBox::Yes | QMessageBox::No);

      if ( responce == QMessageBox::Yes  ) {
        image_sequences_->set(pos, sequence);
        hasChanges = true;
      }
    }
  }

  if ( hasChanges ) {
    sequencesTreeView->refresh();
  }
#endif
}

void MainWindow::onViewInputOptions()
{
  if( !inputOptionsDlgBox ) {

    inputOptionsDlgBox = new QInputOptionsDialogBox(this);
    inputOptionsDlgBox->setInputOptions(inputSourceView->inputOptions());

    connect(inputOptionsDlgBox, &QInputOptionsDialogBox::visibilityChanged,
        viewInputOptionsAction, &QAction::setChecked);

    connect(inputOptionsDlgBox, &QInputOptionsDialogBox::parameterChanged,
        inputSourceView, &QInputSourceView::reloadCurrentFrame);

  }

  if ( !inputOptionsDlgBox->isVisible() ){
    inputOptionsDlgBox->show();
  }
  else {
    inputOptionsDlgBox->hide();
  }
}


void MainWindow::setupInputSequenceView()
{
  QToolBar * toolbar;
  QShortcut * shortcut;

  ///////////////////////////////////////////////////////////////////////

  toolbar = inputSourceView->toolbar();

  static QIcon badframeIcon;
  if( badframeIcon.isNull() ) {
    badframeIcon.addPixmap(getPixmap(ICON_frame), QIcon::Normal, QIcon::Off);
    badframeIcon.addPixmap(getPixmap(ICON_badframe), QIcon::Normal, QIcon::On);
  }

  toolbar->addAction(selectPreviousFileAction_);
  toolbar->addAction(selectNextFileAction);
  toolbar->addAction(reloadCurrentFileAction);

  toolbar->addSeparator();

  toolbar->addAction(createAction(getIcon(ICON_dislike),
      "Bad",
      "Move current image to the .bads subfolder (Ctrl+DEL)",
      [this]() {
        thumbnailsView->moveToBads(imageView->currentFileName());
      },
      new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Delete),
          imageView, nullptr, nullptr,
          Qt::WindowShortcut)));

  toolbar->addAction(badframeAction =
      createCheckableAction(badframeIcon,
          "Bad Frame",
          "Mark / Unmark current frame as bad (Ctrl+W)",
          false,
          [this](bool checked) {

            const c_input_source::sptr & currentSource = inputSourceView->currentSource();
            if ( currentSource ) {
              currentSource->set_badframe(std::max(0, currentSource->curpos() - 1), checked);
              currentSource->save_badframes();
            }

          },
          badframeActionShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_W), this)));


  toolbar->addAction(setReferenceFrameAction =
      createAction(getIcon(ICON_reference),
          "Make reference",
          "Make this frame reference",
          [this]() {

            if ( !is_visible(inputSourceView) || !is_visible(sequencesTreeView) ) {
              return;
            }

            c_image_sequence::sptr selectedSequence;

            const c_input_source::sptr selectedSource =
                sequencesTreeView->getCurrentInputSource(&selectedSequence);

            if ( !selectedSource || !selectedSequence ) {
              return;
            }

            const c_image_processing_pipeline::sptr & currentPipeline =
                selectedSequence->current_pipeline();

            if ( !currentPipeline ) {

              QMessageBox::warning(this, "warning",
                  "No current pipeline is selected,\n "
                  "Master frame is not assigned");

              return;
            }

            if ( !currentPipeline->has_master_frame() ) {

              QMessageBox::warning(this, "warning",
                  "Current pipeline don not use of masster frames,\n"
                  "master frame is not assigned");
              return;
            }


            const c_input_source::sptr & currentSource =
                inputSourceView->currentSource();

            if ( !currentSource ) {
              return;
            }

            const int source_index =
                selectedSequence->indexof(currentSource->filename());

            if ( source_index < 0 ) {
              QMessageBox::warning(this, "warning",
                  "Current source is not in selected sequence,\n"
                  "master frame is not assigned");
              return;
            }

            currentPipeline->set_master_source(selectedSource->filename());
            currentPipeline->set_master_frame_index(std::max(0, currentSource->curpos()-1));
            saveCurrentWork();

          }));

  toolbar->addSeparator();

  toolbar->addWidget(currentFileNameLabel_ctl = new QLabel(""));
  currentFileNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);

  connect(inputSourceView, &QInputSourceView::currentFileNameChanged,
      [this, toolbar]() {
        const QString abspath = inputSourceView->currentFileName();
        currentFileNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
        toolbar->adjustSize();

        if ( is_visible(mtfControl) ) {
          onMtfControlVisibilityChanged(true);
        }
      });

  ///////////////////////////////////////////////////////////////////////


  toolbar = inputSourceView->imageViewToolbar();


  toolbar->addWidget(imageSizeLabel_ctl = new QLabel(""));
  imageSizeLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);


  toolbar->addSeparator();
  toolbar->addWidget(new QToolbarSpacer());

  toolbar->addWidget(editMaskAction =
      createCheckableToolButtonWithContextMenu(getIcon(ICON_mask), "tb1",
          "test",
          false,
          [this](QToolButton * tb) {
            showImageViewOptions(is_visible(imageView) && tb->isChecked());
          },
          [this](QToolButton * tb, const QPoint & pos) {

            if ( is_visible(imageView) && !is_visible(imageViewOptionsDlgBox) ) {

              QMenu menu;
              QAction * action;

              menu.addAction(createCheckableAction(QIcon(), "Transparent mask",
                      "",
                      imageView->transparentMask(),
                      [this](bool checked) {
                        imageView->setTransparentMask(checked);
                      }));

              menu.addSeparator();

              for ( const auto * m = members_of<QImageViewer ::DisplayType>(); !m->name.empty(); ++m ) {

                const QImageViewer ::DisplayType value = (QImageViewer ::DisplayType )m->value;

                menu.addAction(createCheckableAction(QIcon(), m->name.c_str(),
                        "",
                        imageView->displayType() == value,
                        [this, value](bool checked) {
                          imageView->setDisplayType(value);
                        }));

              }

              menu.exec(tb->mapToGlobal(QPoint(pos.x()-2, pos.y()-2)));

            }

          }));


  toolbar->addWidget(shapes_ctl =
      new QShapesButton(imageView->sceneView(),
          this));

  //toolbar->addAction(showMtfControlAction_);
  toolbar->addWidget(createMtfControlButton());

  ///

  _roiActionsMenu.addAction(showRoiOptionsAction =
      createCheckableAction(QIcon(),
          "ROI Options..",
          "Configure ROI rectangle options",
          is_visible(_roiOptionsDialogBox),
          [this](bool checked) {

            if ( !checked ) {
              if ( _roiOptionsDialogBox ) {
                _roiOptionsDialogBox->setVisible(false);
              }
            }
            else {

              if ( !_roiOptionsDialogBox ) {

                _roiOptionsDialogBox = new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
                    imageView->roiShape(),
                    this);

                _roiOptionsDialogBox->loadSettings("QGraphicsRectShapeSettings");

                connect(_roiOptionsDialogBox, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
                    [this](bool visible) {
                      showRoiOptionsAction->setChecked(visible);
                      if ( visible ) {
                        imageView->roiShape()->setVisible(true);
                      }
                    });

              }

              _roiOptionsDialogBox->setVisible(checked);
            }
          }));


  _roiActionsMenu.addAction(showMeasuresSettingsAction);
  _roiActionsMenu.addAction(showMeasuresDisplayAction);
  _roiActionsMenu.addAction(showMeasuresGraphAction);


  toolbar->addWidget(createToolButtonWithPopupMenu(showRoiRectangleAction =
      createCheckableAction(getIcon(ICON_roi),
          "ROI Rectangle",
          "Show / Hide ROI rectangle",
          imageView->roiShape()->isVisible(),
          [this](bool checked) {
            imageView->roiShape()->setVisible(checked);
          }),
      &_roiActionsMenu));


  connect(imageView->roiShape(), &QGraphicsObject::visibleChanged,
      [this]() {
        onWriteDisplayVideo();
        showRoiRectangleAction->setChecked(imageView->roiShape()->isVisible());
      });

  connect(imageView->roiShape(), &QGraphicsShape::itemChanged,
      [this]() {

        onWriteDisplayVideo();

        QGraphicsRectShape * shape =
            imageView->roiShape();

        const QRectF rc =
            shape->sceneRect();

        const QPointF p1 = rc.topLeft();
        const QPointF p2 = rc.bottomRight();
        const QPointF center = rc.center();
        const double width = rc.width();
        const double height = rc.height();

        if ( !statusbarShapesLabel_ctl->isVisible() ) {
          statusbarShapesLabel_ctl->setVisible(true);
        }

        statusbarShapesLabel_ctl->setText(
            qsprintf("ROI: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                p1.x(), p1.y(),
                p2.x(), p2.y(),
                width, height,
                center.x(), center.y()));

        updateMeasurements();
      });

  connect(imageView->roiShape(), &QGraphicsShape::visibleChanged,
      [this]() {
        onWriteDisplayVideo();
        if ( statusbarShapesLabel_ctl ) {
          statusbarShapesLabel_ctl->setVisible(false);
        }
      });

  ///

  toolbar->addWidget(scaleSelection_ctl = new QScaleSelectionButton(this));
  scaleSelection_ctl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(scaleSelection_ctl, &QScaleSelectionButton::scaleChanged,
      [this](int v) {
        imageView->setViewScale(v);
      });

  connect(imageView, &QImageSourceView::onScaleChanged,
      this, &ThisClass::onWriteDisplayVideo);

  connect(imageView, &QImageSourceView::onViewScrolled,
      this, &ThisClass::onWriteDisplayVideo);


  connect(imageView, &QImageFileEditor::onMouseMove,
      [this](QMouseEvent * e) {
        statusbarMousePosLabel_ctl->setText(imageView->statusStringForPixel(e->pos()));
      });

  connect(imageView->scene(), &QImageScene::graphicsItemChanged,
      [this](QGraphicsItem * item) {

        onWriteDisplayVideo();

        QGraphicsLineShape * lineShape = nullptr;
        QGraphicsRectShape * rectShape = nullptr;

        if ( (lineShape = dynamic_cast<QGraphicsLineShape * >(item)) ) {

          const QLineF line = lineShape->sceneLine();

          const QPointF p1 = line.p1();
          const QPointF p2 = line.p2();
          const double length = hypot(p2.x()-p1.x(), p2.y()-p1.y());
          const double angle = atan2(p2.y()-p1.y(), p2.x()-p1.x());

          if ( !statusbarShapesLabel_ctl->isVisible() ) {
            statusbarShapesLabel_ctl->setVisible(true);
          }

          statusbarShapesLabel_ctl->setText(
              qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
                  p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));

          updateProfileGraph(item);

        }
        else if ( (rectShape = dynamic_cast<QGraphicsRectShape* >(item))) {

          const QRectF rc = rectShape->mapToScene(rectShape->rect()).boundingRect();

          const QPointF p1 = rc.topLeft();
          const QPointF p2 = rc.bottomRight();
          const QPointF center = rc.center();
          const double width = rc.width();
          const double height = rc.height();

          if ( !statusbarShapesLabel_ctl->isVisible() ) {
            statusbarShapesLabel_ctl->setVisible(true);
          }

          statusbarShapesLabel_ctl->setText(
              qsprintf("RECT: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                  p1.x(), p1.y(),
                  p2.x(), p2.y(),
                  width, height,
                  center.x(), center.y()));

        }
      });

  connect(imageView->scene(), &QImageScene::graphicsItemVisibleChanged,
      [this]() {
        onWriteDisplayVideo();
        if ( statusbarShapesLabel_ctl->isVisible() ) {
          statusbarShapesLabel_ctl->setVisible(false);
        }
      });

  connect(imageView->scene(), &QImageScene::graphicsItemDestroyed,
      [this]() {
        onWriteDisplayVideo();
        if ( statusbarShapesLabel_ctl->isVisible() ) {
          statusbarShapesLabel_ctl->setVisible(false);
        }
      });



  ///////////////////////////////////////////////////////////////////////


  toolbar = inputSourceView->cloudViewToolbar();

//  toolbar->addAction(showMtfControlAction_);
  toolbar->addWidget(createMtfControlButton());

  toolbar->addWidget(createToolButton(getIcon(ICON_options),
      "Options",
      "Cloud View Options...",
      [this](QToolButton * tb) {

        QMenu menu;

        menu.addAction(createMenuWidgetAction<QSpinBox>("Point size: ",
                nullptr,
                [this](const auto * action) {
                  action->icon()->setPixmap(getPixmap(ICON_point_size)); // .scaled(QSize(16,16))
                  QSpinBox * spinBox = action->control();
                  spinBox->setKeyboardTracking(false);
                  spinBox->setRange(1, 32);
                  spinBox->setValue((int)cloudView->pointSize());
                  connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                      [this](int value) {
                    cloudView->setPointSize(value);
                      });
                }));

        menu.addAction(createMenuWidgetAction<QSpinBox>("Point brightness: ",
                nullptr,
                [this](const auto * action) {
                  action->icon()->setPixmap(getPixmap(ICON_brightness)); // .scaled(QSize(16,16))
                  QSpinBox * spinBox = action->control();
                  spinBox->setKeyboardTracking(false);
                  spinBox->setRange(-128, 128);
                  spinBox->setValue((int)cloudView->pointBrightness());
                  connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                      [this](int value) {
                        if ( is_visible(cloudView) ) {
                          cloudView->setPointBrightness(value);
                        }
                      });
                }));

        menu.addAction(createMenuWidgetAction<QColorPickerButton>("Background color: ",
                nullptr,
                [this](const auto * action) {
                  action->icon()->setPixmap(getPixmap(ICON_bgcolor)); // .scaled(QSize(16,16))
                  QColorPickerButton * colorPicker = action->control();
                  colorPicker->setColor(cloudView->backgroundColor());
                  connect(colorPicker, &QColorPickerButton::colorSelected,
                      [this, colorPicker]() {
                        if ( cloudView ) {
                          cloudView->setBackgroundColor(colorPicker->color());
                        }
                      });
                }));

        menu.addAction(createAction(getIcon(ICON_cloud_rotate),
                "Show cloud center",
                "Rotate camera to show point cloud center",
                [this]() {
                  if ( is_visible(cloudView) ) {
                    cloudView->rotateToShowCloud();
                  }
                }));

        menu.addAction(createCheckableAction(getIcon(ICON_cloud_view_XYZ),
                "Show/Hide Main coordinate axes",
                "Show/Hide Main coordinate axes",
                cloudView->showMainAxes(),
                [this](bool checked) {
                  if ( is_visible(cloudView) ) {
                    cloudView->setShowMainAxes(checked);
                  }
                }));


        menu.addAction(createCheckableAction(getIcon(ICON_cloud_view_target),
                "Auto Show Target point",
                "",
                cloudView->autoShowViewTarget(),
                [this](bool checked) {
                  if ( is_visible(cloudView) ) {
                    cloudView->setAutoShowViewTarget(checked);
                  }
                }));


        if ( !cloudView->grids().empty() ) {

            menu.addAction(createCheckableAction(getIcon(ICON_cloud_view_grid),
                  "Planar grid options...",
                  "Open Planar grid options modal dialog box",
                  is_visible(_glGridSettingsDialog),
                  [this]() {

                    if ( !_glGridSettingsDialog ) {

                      _glGridSettingsDialog =
                          new QGLViewPlanarGridSettingsDialogBox("Planar grid options...",
                              this);

                      connect(_glGridSettingsDialog, &QGLViewPlanarGridSettingsDialogBox::parameterChanged,
                          [this]() {
                            cloudView->update();
                            cloudView->saveSettings();
                          });
                    }

                    if ( !_glGridSettingsDialog->isVisible() ) {
                      _glGridSettingsDialog->setOptions(&cloudView->grids().front());
                      _glGridSettingsDialog->show();
                      _glGridSettingsDialog->raise();
                    }
                    else {
                      _glGridSettingsDialog->setOptions(nullptr);
                      _glGridSettingsDialog->hide();
                    }
                  }));
        }

        if ( !showCloudSettingsDialogBoxAction ) {

          showCloudSettingsDialogBoxAction =
            createCheckableAction(getIcon(ICON_list),
                "Clouds options...",
                "Show / Hide individual clouds options",
                is_visible(cloudSettingsDialogBox),
                this,
                &ThisClass::onShowCloudSettingsDialogBoxActionClicked);
        }

        menu.addAction(showCloudSettingsDialogBoxAction);


        menu.addSeparator();

        if ( !showCloudViewSettingsDialogBoxAction ) {

          showCloudViewSettingsDialogBoxAction =
            createCheckableAction(getIcon(ICON_options),
                "Advanced ...",
                "Show advanced options",
                is_visible(cloudViewSettingsDialogBox),
                this,
                &ThisClass::onShowCloudViewSettingsDialogBoxActionClicked);
        }

        menu.addAction(showCloudViewSettingsDialogBoxAction);

        menu.exec(tb->mapToGlobal(QPoint(tb->width() - 4,tb->height() - 4)));
      }));



  static const auto uncheckOtherActions =
      [](const QList<QAction*> & actions, const QAction * checkedAction) {
        if ( checkedAction->isChecked() ) {
          for ( QAction * a : actions ) {
            if ( a != checkedAction ) {
              a->setChecked(false);
            }
          }
        }
      };


  _pointSelectionModeActions.append(createCheckableAction2(QIcon(), "3D Ruler Line",
      "3D ruler line selection", false,
      [this](QAction * action) {

        uncheckOtherActions(_pointSelectionModeActions, action);

        if ( !action->isChecked() ) {
          inputSourceView->setPointSelectionMode(nullptr);
          _pointSelection3DRulerMode.disconnect(this);
        }
        else {

          connect(&_pointSelection3DRulerMode, &QPointSelection3DRulerMode::rulerChanged,
              [this]() {
                if ( is_visible(logWidget_ctl) ) {

                  const QGLLineShape & line = _pointSelection3DRulerMode.ruler();
                  const QVector3D & start = line.start();
                  const QVector3D & end = line.end();

                  const QString text = qsprintf("RULER: start=(%+g %+g %+g) end=(%+g %+g %+g) length=%g",
                      start.x(), start.y(), start.z(),
                      end.x(), end.y(), end.z(),
                      QVector3D(end-start).length());

                  logWidget_ctl->putText(text);
                }

              });


          inputSourceView->setPointSelectionMode(&_pointSelection3DRulerMode);
        }

      }));


  toolbar->addWidget(_pointSelectionModeToolbutton =
      createPointSelectionModeToolButton(this));

  _pointSelectionModeMenu.addActions(_pointSelectionModeActions);
  _pointSelectionModeToolbutton->setMenu(&_pointSelectionModeMenu);




  connect(cloudView, &QPointCloudSourceView::glPointMouseEvent,
      [this](const QPointF & mousePos, QEvent::Type mouseEventType, Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
          bool objHit, double objX, double objY, double objZ) {
          //statusbarMousePosLabel_ctl->setText(cloudView->statusStringForPoint(cloud_index, point_index));
        CF_DEBUG("FIXME: Unhandled mouse event");
      });


  ///////////////////////////////////////////////////////////////////////


  toolbar = inputSourceView->textViewToolbar();
  toolbar->addAction(showFindTextDialogAction =
        createCheckableAction(getIcon(ICON_search),
            "Find Text...",
            "Find text in text file",
            false,
            this,
            &ThisClass::toggleFindTextDialogBox,
            new QShortcut(QKeySequence::Find,
                textView, nullptr, nullptr,
                Qt::WidgetWithChildrenShortcut/*WindowShortcut*/)
                ));

  findNextShortcut = new QShortcut(QKeySequence::FindNext,
      textView, [this]() {
        if ( is_visible(textView) ) {
          textView->findNext();
        }
      },
      Qt::WidgetWithChildrenShortcut);

  findPrevShortcut = new QShortcut(QKeySequence::FindPrevious,
      textView, [this]() {
        if ( is_visible(textView) ) {
          textView->findPrevious();
        }
      },
      Qt::WidgetWithChildrenShortcut);







  ///////////////////////////////////////////////////////////////////////

  toolbar = inputSourceView->rightToolbar();
  // toolbar->addSeparator();

  toolbar->addWidget(_displayImageVideoWriterToolButton =
      createDisplayVideoWriterOptionsToolButton(&_diplayImageWriter, this));

  connect(&_diplayImageWriter, &QDisplayVideoWriter::stateChanged,
      [this]() {
        if ( _diplayImageWriter.started() ) {
          connect(cloudView, &QGLView::displayImageChanged,
              this, &ThisClass::onCurrentViewDisplayImageChanged);
        }
        else {
          disconnect(cloudView, &QGLView::displayImageChanged,
              this, &ThisClass::onCurrentViewDisplayImageChanged);
        }
      });



  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          inputSourceView, nullptr, nullptr,
          Qt::WindowShortcut)));



  ///////////////////////////////////////////////////////////////////////
}

void MainWindow::setupPipelineProgressView()
{
  QToolBar * toolbar =
      pipelineProgressImageView->embedToolbar();

  addStretch(toolbar);

  QLabel * imageSizeLabel_ctl = new QLabel("");
  imageSizeLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);
  toolbar->addWidget(imageSizeLabel_ctl);
  connect(pipelineProgressImageView, &QProgressImageViewer::currentImageChanged,
      [this, imageSizeLabel_ctl]() {
        imageSizeLabel_ctl->setText(toQString(pipelineProgressImageView->currentImage().size()));
      });

  toolbar->addSeparator();

  QScaleSelectionButton * scaleSelection_ctl = new QScaleSelectionButton(this);
  scaleSelection_ctl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  toolbar->addWidget(scaleSelection_ctl);
  connect(scaleSelection_ctl, &QScaleSelectionButton::scaleChanged,
      [this](int v) {
        pipelineProgressImageView->setViewScale(v);
      });


//  toolbar->addAction(showMtfControlAction_);
  toolbar->addWidget(createMtfControlButton());


  toolbar->addSeparator();

  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          pipelineProgressImageView, nullptr, nullptr,
          Qt::WindowShortcut)));


  connect(pipelineProgressImageView, &QImageFileEditor::onMouseMove,
      [this](QMouseEvent * e) {
        statusbarMousePosLabel_ctl->setText(pipelineProgressImageView->statusStringForPixel(e->pos()));
      });

}


void MainWindow::toggleFindTextDialogBox(bool fshow)
{
  if ( !fshow ) {
    if ( is_visible(findTextDialog) ) {
      findTextDialog->hide();
    }
  }
  else {
    if ( !findTextDialog ) {

      findTextDialog = new QFindTextDialog(this);

      connect(findTextDialog, &QFindTextDialog::visibilityChanged,
          showFindTextDialogAction, &QAction::setChecked);

      connect(findTextDialog, &QFindTextDialog::findTextRequested,
          [this](const QString &text, bool caseSensitive, bool wholeWords, bool backward) {
            if ( is_visible(textView) ) {
              textView->findString(text, caseSensitive, wholeWords, backward);
            }
          });
    }

    if ( !findTextDialog->isVisible() ) {
      findTextDialog->show();
    }

    findTextDialog->raise();
  }

}

///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
