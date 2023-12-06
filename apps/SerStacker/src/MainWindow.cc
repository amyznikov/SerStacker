/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
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
#include <core/io/load_image.h>
#include <core/debug.h>

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
#define ICON_mask               ":/gui/icons/mask"
#define ICON_frame              ":/gui/icons/frame"
#define ICON_badframe           ":/gui/icons/badframe"

#define ICON_copy               ":/gui/icons/copy"
#define ICON_delete             ":/gui/icons/delete"

#define ICON_roi                ":/serstacker/icons/roi.png"
#define ICON_point_size         ":/serstacker/icons/degree.png"
#define ICON_brightness         ":/serstacker/icons/brightness.png"
#define ICON_bgcolor            ":/serstacker/icons/fill-color.png"

#define ICON_cloud_rotate       ":/serstacker/icons/cloud_rotate.png"
#define ICON_cloud_view_target  ":/serstacker/icons/cloud_view_target.png"


namespace serstacker {
///////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/serstacker/icons/app-icon.png"));
  updateWindowTittle();
  QImageProcessorsCollection::load();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));
  centralStackedWidget->addWidget(pipelineOptionsView = new QPipelineOptionsView(this));

  centralStackedWidget->addWidget(inputSequenceView =
      new QInputSequenceView(this,
          imageView = new QSerStackerImageEditor(this),
          cloudView = new QCloudViewer(this),
          textView = new QTextFileViewer(this)));

  connect(centralStackedWidget, &QStackedWidget::currentChanged,
      [this]() {

        if ( is_visible(cloudViewSettingsDialogBox) ) {
          if ( ! cloudView->isVisible() ) {
            cloudViewSettingsDialogBox->hide();
          }
        }

        if ( is_visible(mtfControl_) ) {
          // force update MTF histogram
          onMtfControlVisibilityChanged(true);
        }

    });

  connect(inputSequenceView, &QInputSequenceView::currentViewChanged,
      this, & ThisClass::onCurrentViewVisibilityChanged);

  connect(imageView, &QImageEditor::displayImageChanged,
      this, &ThisClass::onCurrentViewDisplayImageChanged);

  connect(cloudView, &QCloudViewer::displayImageChanged,
      this, &ThisClass::onCurrentViewDisplayImageChanged);

  connect(imageView, &QImageEditor::currentImageChanged,
      this, &ThisClass::onImageViewCurrentImageChanged);


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
  setupMeasures();
  setupProfileGraph();
  setupInputSequenceView();



  tabifyDockWidget(fileSystemTreeDock, sequencesTreeViewDock);
  //tabifyDockWidget(sequencesTreeDock, imageProcessorSelectorDock);
  tabifyDockWidget(sequencesTreeViewDock, imageProcessorDock_);


  connect(QPipelineThread::instance(), &QPipelineThread::started,
      this, &ThisClass::onPipelineThreadStarted);

  connect(QPipelineThread::instance(), &QPipelineThread::finished,
      this, &ThisClass::onPipelineThreadFinished);


  restoreState();

  imageView->set_current_processor(imageProcessor_ctl->current_processor());
}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("SerStacker");
}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);

  if( fileSystemTreeDock ) {
    settings.setValue("fileSystemTree/absoluteFilePath",
        fileSystemTreeDock->currentAbsoluteFilePath());
  }

  if ( cloudView ) {
    cloudView->saveParameters();
  }
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->displayPath(settings.value(
        "fileSystemTree/absoluteFilePath").toString());
  }

  if ( cloudView ) {
    cloudView->loadParameters();
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

  menuFile_->addAction(reloadCurrentFileAction =
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

  menuFile_->addAction(selectPreviousFileAction_ =
      createAction(getIcon(ICON_prev),
          "Previous (Ctrl+PgUp)",
          "Select previous file (Ctrl+PgUp)",
          [this]() {
            thumbnailsView->selectPrevIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile_->addAction(selectNextFileAction =
      createAction(getIcon(ICON_next),
          "Next (Ctrl+PgDown)",
          "Select next file (Ctrl+PgDown)",
          [this]() {
            thumbnailsView->selectNextIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile_->addSeparator();

  saveImageAsAction =
      menuFile_->addAction("Save current image as...",
          this, &ThisClass::onSaveCurrentImageAs);

  saveImageAsAction->setEnabled(is_visible(imageView) &&
      !imageView->currentImage().empty());

  saveDisplayImageAsAction =
      menuFile_->addAction("Save current display image as...",
          this, &ThisClass::onSaveCurrentDisplayImageAs);

  saveDisplayImageAsAction->setEnabled(is_visible(imageView) &&
      !imageView->displayImage().empty());

  saveImageMaskAction =
      menuFile_->addAction("Save current image mask...",
          this, &ThisClass::onSaveCurrentImageMask);

  saveImageMaskAction->setEnabled(is_visible(imageView) &&
      !imageView->currentMask().empty());

  loadImageMaskAction =
      menuFile_->addAction("Set current image mask from file...",
          this, &ThisClass::onLoadCurrentImageMask);

  loadImageMaskAction->setEnabled(is_visible(imageView) &&
      !imageView->currentImage().empty());

  menuFile_->addSeparator();

  loadStackAction =
      menuFile_->addAction("Load stack config...",
          this, &ThisClass::onLoadStackConfig);


  menuFile_->addSeparator();

  quitAppAction =
      menuFile_->addAction("Quit",
          this, &ThisClass::close);


  //
  // Edit
  //

  menuEdit_->addAction(copyDisplayImageAction =
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
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuEdit_->addAction(copyDisplayViewportAction =
      createAction(QIcon(),
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          [this]() {
            if ( is_visible(cloudView) ) {
              cloudView->copyViewportToClipboard();
            }
            else if ( is_visible(imageView) ) {
              QPixmap pxmap = imageView->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));


  //
  // View
  //
  menuView_->addAction(viewInputOptionsAction =
      createCheckableAction(QIcon(),
          "Input Options...",
          "Configure general input options",
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
  statusbarShowLog_ctl->setDefaultAction(showLogWidgetAction_);
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
          menuView_);

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
      addImageSequenceTreeDock(this,
          Qt::LeftDockWidgetArea,
          "sequencesTreeDock",
          "Sequences",
          menuView_);

  sequencesTreeView =
      sequencesTreeViewDock->treeView();

  sequencesTreeView->loadSequences();

  connect(sequencesTreeView, &QImageSequencesTree::currentItemChanged,
      this, &ThisClass::onStackTreeCurrentItemChanged);

  connect(sequencesTreeView, &QImageSequencesTree::itemDoubleClicked,
      this, &ThisClass::onStackTreeItemDoubleClicked);

  connect(sequencesTreeView, &QImageSequencesTree::showImageSequenceOptionsClicked,
      this, &ThisClass::onShowImageSequenceOptions);

  connect(sequencesTreeView, &QImageSequencesTree::imageSequenceCollectionChanged,
        this, &ThisClass::saveCurrentWork );

  connect(sequencesTreeView, &QImageSequencesTree::imageSequenceSourcesChanged,
      [this](const c_image_sequence::sptr & sequence) {
        if ( pipelineOptionsView->isVisible() && pipelineOptionsView->current_sequence() == sequence ) {
          // fixme: temporary hack to force update options views
          pipelineOptionsView->set_current_sequence(sequence);
        }
        saveCurrentWork();
  });

  connect(sequencesTreeView, &QImageSequencesTree::imageSequenceNameChanged,
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

  if( is_visible(inputSequenceView) ) {

    const c_input_sequence::sptr &currentSequence =
        inputSequenceView->currentSequence();

    if( currentSequence ) {

      c_input_source::sptr currentSource =
          currentSequence->current_source();

      if( currentSource ) {
        isBadFrameSelected =
            currentSource->is_badframe(currentSequence->current_pos() - 1);
      }
    }
  }

  if( badframeAction->isChecked() != isBadFrameSelected ) {
    badframeAction->setChecked(isBadFrameSelected);
  }
}

void MainWindow::onWriteDisplayVideo()
{
  if( !diplayImageWriter_.started() || diplayImageWriter_.paused() ) {
    return;
  }

  QWidget * currentView =
      inputSequenceView->currentView();

  if( currentView == imageView ) {

    if( !diplayImageWriter_.writeViewPort() ) {
      if( !imageView->displayImage().empty() ) {
        diplayImageWriter_.write(imageView->displayImage());
      }
    }
    else {

      QImage image =
          imageView->grabViewportPixmap().toImage().convertToFormat(
              QImage::Format_BGR888);

      if( !image.isNull() ) {

        diplayImageWriter_.write(cv::Mat(image.height(), image.width(), CV_8UC3,
            (void*) image.constBits(),
            image.bytesPerLine()));
      }

    }
  }

  else if( currentView == cloudView ) {

    QImage image =
        cloudView->grabViewportPixmap().toImage().convertToFormat(
            QImage::Format_BGR888);

    if( !image.isNull() ) {

      diplayImageWriter_.write(cv::Mat(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine()));
    }
  }
}

void MainWindow::onCurrentViewVisibilityChanged()
{
  if( is_visible(cloudViewSettingsDialogBox) && !is_visible(cloudView) ) {
    cloudViewSettingsDialogBox->hide();
  }

  if( is_visible(mtfControl_) ) { // force update MTF histogram
    onMtfControlVisibilityChanged(true);
  }

  checkIfBadFrameSelected();


  QWidget * currentView =
      inputSequenceView->currentView();

  if ( currentView == imageView ) {
    if ( !imageView->isVisible() ) {
      saveImageAsAction->setEnabled(false);
      saveDisplayImageAsAction->setEnabled(false);
      copyDisplayImageAction->setEnabled(false);
      copyDisplayViewportAction->setEnabled(false);
      saveImageMaskAction->setEnabled(false);
      loadImageMaskAction->setEnabled(false);
      displayImageVideoWriterToolButton_->setEnabled(false);
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
      displayImageVideoWriterToolButton_->setEnabled(true);
    }

  }
  else  if ( currentView == cloudView ) {
    saveImageAsAction->setEnabled(false);
    saveImageMaskAction->setEnabled(false);
    loadImageMaskAction->setEnabled(false);

    if ( !cloudView->isVisible() ) {
      saveDisplayImageAsAction->setEnabled(false);
      copyDisplayImageAction->setEnabled(false);
      copyDisplayViewportAction->setEnabled(false);
      displayImageVideoWriterToolButton_->setEnabled(false);
    }
    else {
      saveDisplayImageAsAction->setEnabled(true);
      copyDisplayImageAction->setEnabled(true);
      copyDisplayViewportAction->setEnabled(true);
      displayImageVideoWriterToolButton_->setEnabled(true);
    }
  }
  else {
    saveImageAsAction->setEnabled(false);
    saveDisplayImageAsAction->setEnabled(false);
    copyDisplayImageAction->setEnabled(false);
    copyDisplayViewportAction->setEnabled(false);
    saveImageMaskAction->setEnabled(false);
    loadImageMaskAction->setEnabled(false);
    displayImageVideoWriterToolButton_->setEnabled(false);
  }

}


void MainWindow::onCurrentViewDisplayImageChanged()
{
  QWidget * currentView =
      inputSequenceView->currentView();

  if ( currentView == imageView ) {

    const bool isvisible =
        imageView->isVisible();

    const bool hasdisplayimage =
        !imageView->displayImage().empty();

    saveDisplayImageAsAction->setEnabled(isvisible && hasdisplayimage);
    copyDisplayImageAction->setEnabled(isvisible && hasdisplayimage);
    copyDisplayViewportAction->setEnabled(isvisible);

    if ( diplayImageWriter_.started() ) {

      if ( !isvisible ) {
        diplayImageWriter_.stop();
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

    if( diplayImageWriter_.started() ) {

      if( !isvisible ) {
        diplayImageWriter_.stop();
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

  if( is_visible(mtfControl_) ) {
    onMtfControlVisibilityChanged(true);
  }

}


void MainWindow::onImageProcessorParameterChanged()
{
  Base::onImageProcessorParameterChanged();
  imageView->set_current_processor(imageProcessor_ctl->current_processor());
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  if( !visible ) {
    mtfControl_->setMtfDisplaySettings(nullptr);
  }
  else {

    if( is_visible(imageView) ) {
      mtfControl_->setMtfDisplaySettings(imageView->mtfDisplayFunction());
    }
    else if( is_visible(cloudView) ) {
      mtfControl_->setMtfDisplaySettings(&cloudView->mtfDisplay());
    }
    else {
      mtfControl_->setMtfDisplaySettings(nullptr);
    }

    const QString currentFileName =
        mtfControl_->mtfDisplaySettings() ?
            QFileInfo(inputSequenceView->currentFileName()).fileName() :
            "";

    if( currentFileName.isEmpty() ) {
      mtfControl_->setWindowTitle("Adjust Display Levels ...");
    }
    else {
      mtfControl_->setWindowTitle(qsprintf("Adjust Display Levels: %s",
          currentFileName.toUtf8().constData()));
    }
  }
}

void MainWindow::onShowCloudViewSettingsDialogBoxActionClicked(bool checked)
{
  if ( checked && !cloudViewSettingsDialogBox ) {

    cloudViewSettingsDialogBox = new QCloudViewSettingsDialogBox(this);
    cloudViewSettingsDialogBox->setCloudViewer(cloudView);
    connect(cloudViewSettingsDialogBox, &QCloudViewSettingsDialogBox::visibilityChanged,
        showCloudViewSettingsDialogBoxAction, &QAction::setChecked);
  }

  if ( cloudViewSettingsDialogBox ) {
    if ( !checked ) {
      cloudViewSettingsDialogBox->hide();
    }
    else {
      cloudViewSettingsDialogBox->setWindowTitle(QFileInfo(cloudView->currentFileName()).fileName());
      cloudViewSettingsDialogBox->showNormal();
    }
  }
}

void MainWindow::updateMeasurements()
{
  if( !QMeasureProvider::requested_measures().empty() && imageView->roiShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(imageView);

    if ( !imageView->currentImage().empty() ) {

      QMeasureProvider::compute(imageView->currentImage(),
          imageView->currentMask(),
          imageView->roiShape()->iSceneRect());
    }
  }
}

void MainWindow::updateProfileGraph(QGraphicsItem * lineItem)
{
  if( is_visible(profileGraph_ctl_) && is_visible(imageView) ) {

    if( QGraphicsLineShape *lineShape = dynamic_cast<QGraphicsLineShape*>(lineItem) ) {

      profileGraph_ctl_->showProfilePlot(lineShape->sceneLine(),
          imageView->currentImage(),
          imageView->currentMask());

    }
    else {

      QLine line =
          profileGraph_ctl_->currentLine();

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

      profileGraph_ctl_->showProfilePlot(line,
          imageView->currentImage(),
          imageView->currentMask());

    }
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

  centralStackedWidget->setCurrentWidget(inputSequenceView);
  inputSequenceView->openFile(abspath);
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
    centralStackedWidget->setCurrentWidget(inputSequenceView);
    inputSequenceView->openFile(source->filename());
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
  if ( QPipelineThread::isRunning() ) {

    if ( source ) {
      pipelineProgressView->setImageViewer(nullptr);
      centralStackedWidget->setCurrentWidget(inputSequenceView);
      inputSequenceView->openFile(source->filename());
    }
    else if ( sequence ) {

      QWidget * currentCentralWidget =
          centralStackedWidget->currentWidget();

      const c_image_processing_pipeline::sptr & pipeline =
          QPipelineThread::currentPipeline();

      if ( pipeline && sequence == pipeline->input_sequence() ) {

        if ( currentCentralWidget == imageView ) {

          if ( !pipelineProgressView->imageViewer() ) {
            pipelineProgressView->setImageViewer(imageView);
          }
          else {
            pipelineProgressView->setImageViewer(nullptr);
            pipelineOptionsView->set_current_sequence(sequence);
            centralStackedWidget->setCurrentWidget(pipelineOptionsView);
          }

        }
        else if ( currentCentralWidget == pipelineOptionsView ) {
          if ( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
          else {
            centralStackedWidget->setCurrentWidget(inputSequenceView);
            pipelineProgressView->setImageViewer(imageView);
          }
        }
        else {
          centralStackedWidget->setCurrentWidget(inputSequenceView);
          pipelineProgressView->setImageViewer(imageView);
        }
      }
      else {

        if ( currentCentralWidget == pipelineOptionsView ) {
          if ( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
        }
        else {
          pipelineOptionsView->set_current_sequence(sequence);
          centralStackedWidget->setCurrentWidget(pipelineOptionsView);
        }
      }
    }

  }
  else if ( source ) {
    if ( centralStackedWidget->currentWidget() == inputSequenceView ) {
      centralStackedWidget->setCurrentWidget(thumbnailsView);
      thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
    }
    else {
      centralStackedWidget->setCurrentWidget(inputSequenceView);
      inputSequenceView->openFile(source->filename());
    }
  }
  else if ( sequence ) {
    if ( centralStackedWidget->currentWidget() == pipelineOptionsView ) {
      if( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }
    }
    else {
      pipelineOptionsView->set_current_sequence(sequence);
      centralStackedWidget->setCurrentWidget(pipelineOptionsView);
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
  if ( centralStackedWidget->currentWidget() != inputSequenceView ) {
    centralStackedWidget->setCurrentWidget(inputSequenceView);
  }

  imageView->clear();
  pipelineProgressView->setImageViewer(imageView);
  inputSequenceView->showImageView();

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
              imageView->currentFileName());

      if( !savedFileName.isEmpty() ) {
        imageView->setCurrentFileName(savedFileName);
      }
    }
  }
  else if( is_visible(cloudView) ) {

    QImage image =
        cloudView->grabViewportPixmap().toImage().convertToFormat(
            QImage::Format_BGR888);

    if( !image.isNull() ) {

      const cv::Mat M(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine());

      saveImageFileAs(this, M, cv::Mat());
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
          imageView->currentFileName());
    }
  }
  else if( is_visible(cloudView) ) {

    QImage image =
        cloudView->grabViewportPixmap().toImage().convertToFormat(
            QImage::Format_BGR888);

    if( !image.isNull() ) {

      const cv::Mat displayImage(image.height(), image.width(), CV_8UC3,
          (void*) image.constBits(),
          image.bytesPerLine());

      saveImageFileAs(this,
          displayImage,
          cv::Mat());
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
          QString("%1.mask.png").arg(imageView->currentFileName()));
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
  // image_sequences_->save();
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
    inputOptionsDlgBox = new QGeneralAppSettingsDialogBox(this);
    inputOptionsDlgBox->setInputSequenceView(inputSequenceView);
    connect(inputOptionsDlgBox, &QGeneralAppSettingsDialogBox::visibilityChanged,
        viewInputOptionsAction, &QAction::setChecked);
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

  ///////////////////////////////////////////////////////////////////////

  toolbar = inputSequenceView->toolbar();

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
          "Mark / Unmark current frame as bad (Ctrl+A)",
          false,
          [this](bool checked) {
            if ( is_visible(inputSequenceView) ) {

              const c_input_sequence::sptr & currentSequence =
                  inputSequenceView->currentSequence();

              if ( currentSequence ) {

                c_input_source::sptr currentSource =
                    currentSequence->current_source();

                if ( currentSource ) {
                  currentSource->set_badframe(currentSequence->current_pos() - 1, checked);
                  currentSource->save_badframes();
                }
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_A),
              imageView, nullptr, nullptr,
              Qt::WindowShortcut)));


  toolbar->addAction(setReferenceFrameAction =
      createAction(getIcon(ICON_reference),
          "Make reference",
          "Make this frame reference",
          [this]() {
            if ( is_visible(inputSequenceView) && is_visible(sequencesTreeView) ) {

              c_input_source::sptr selectedSource;
              c_image_sequence::sptr selectedSequence;

              selectedSource =
                  sequencesTreeView->getCurrentInputSource(&selectedSequence);

              if ( selectedSource && selectedSequence ) {

                const c_input_sequence::sptr & currentSequence =
                    inputSequenceView->currentSequence();

                if ( currentSequence && currentSequence->current_source()->filename() == selectedSource->filename() ) {

                  const c_image_processing_pipeline::sptr currentPipeline =
                      selectedSequence->current_pipeline();

                  if ( currentPipeline && currentPipeline->has_master_frame() ) {
                    currentPipeline->set_master_source(selectedSource->filename());
                    currentPipeline->set_master_frame_index(currentSequence->current_pos() - 1);
                    saveCurrentWork();
                  }
                  else {

                    QMessageBox::warning(this, "warning",
                        qsprintf("No current pipeline is selected,\n"
                            "master frame is not assigned\n"
                            " %s: %d",
                            __FILE__, __LINE__) );
                  }
                }
              }
            }
          }));

  toolbar->addSeparator();

  toolbar->addWidget(currentFileNameLabel_ctl = new QLabel(""));
  currentFileNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);
  connect(inputSequenceView, &QInputSequenceView::currentFileNameChanged,
      [this]() {
        const QString abspath = inputSequenceView->currentFileName();
        currentFileNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
        if ( is_visible(mtfControl_) ) {
          onMtfControlVisibilityChanged(true);
        }
      });


  // toolbar->addSeparator();

  ///////////////////////////////////////////////////////////////////////


  toolbar = inputSequenceView->imageViewToolbar();


  toolbar->addWidget(imageSizeLabel_ctl = new QLabel(""));
  imageSizeLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);


  toolbar->addSeparator();
  toolbar->addWidget(new QToolbarSpacer());


  toolbar->addAction(editMaskAction =
      createCheckableAction(getIcon(ICON_mask),
          "Mask",
          "View / Edit image mask",
          false,
          [this](bool checked) {
            showImageViewOptions(is_visible(imageView) && checked);
          }));


  toolbar->addWidget(shapes_ctl =
      new QShapesButton(imageView->sceneView(),
          this));

  toolbar->addAction(showMtfControlAction_);


  ///

  roiActionsMenu_.addAction(showRoiOptionsAction =
      createCheckableAction(QIcon(),
          "ROI Options..",
          "Configure ROI rectangle options",
          is_visible(roiOptionsDialogBox_),
          [this](bool checked) {

            if ( !checked ) {
              if ( roiOptionsDialogBox_ ) {
                roiOptionsDialogBox_->setVisible(false);
              }
            }
            else {

              if ( !roiOptionsDialogBox_ ) {

                roiOptionsDialogBox_ = new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
                    imageView->roiShape(),
                    this);

                roiOptionsDialogBox_->loadParameters();

                connect(roiOptionsDialogBox_, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
                    [this](bool visible) {
                      showRoiOptionsAction->setChecked(visible);
                      if ( visible ) {
                        imageView->roiShape()->setVisible(true);
                      }
                    });

              }

              roiOptionsDialogBox_->setVisible(checked);
            }
          }));


  roiActionsMenu_.addAction(showMeasuresSettingsAction_);
  roiActionsMenu_.addAction(showMeasuresDisplayAction_);
  roiActionsMenu_.addAction(showMeasuresGraphAction_);


  toolbar->addWidget(createToolButtonWithPopupMenu(showRoiRectangleAction =
      createCheckableAction(getIcon(ICON_roi),
          "ROI Rectangle",
          "Show / Hide ROI rectangle",
          imageView->roiShape()->isVisible(),
          [this](bool checked) {
            imageView->roiShape()->setVisible(checked);
          }),
      &roiActionsMenu_));


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

  connect(imageView, &QSerStackerImageEditor::onScaleChanged,
      this, &ThisClass::onWriteDisplayVideo);

  connect(imageView, &QSerStackerImageEditor::onViewScrolled,
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


  toolbar = inputSequenceView->cloudViewToolbar();

  toolbar->addAction(showMtfControlAction_);

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

        menu.addAction(createCheckableAction(getIcon(ICON_cloud_view_target),
                "Auto Show Target point",
                "",
                cloudView->autoShowViewTarget(),
                [this](bool checked) {
                  if ( is_visible(cloudView) ) {
                    cloudView->setAutoShowViewTarget(checked);
                  }
                }));

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


  ///////////////////////////////////////////////////////////////////////


  toolbar = inputSequenceView->textViewToolbar();


  ///////////////////////////////////////////////////////////////////////

  toolbar = inputSequenceView->rightToolbar();
  // toolbar->addSeparator();

  diplayImageWriter_.loadParameters();
  toolbar->addWidget(displayImageVideoWriterToolButton_ =
      createDisplayVideoWriterOptionsToolButton(&diplayImageWriter_, this));

  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          inputSequenceView, nullptr, nullptr,
          Qt::WindowShortcut)));



  ///////////////////////////////////////////////////////////////////////
}


///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
