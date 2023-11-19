/*
 * MainWindow.cc
 *
 *  Created at Sat, November 18, 2023
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
#define ICON_cloud_rotate       ":/serstacker/icons/cloud_rotate.png"
#define ICON_cloud_view_target  ":/serstacker/icons/cloud_view_target.png"


namespace cloudview {
///////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/cloudview/icons/app-icon.png"));
  updateWindowTittle();
  QImageProcessorsCollection::load();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));


  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  setupMainMenu();
  setupFileSystemTreeView();
  setupThumbnailsView();
  setupDatasetView();

  tabifyDockWidget(fileSystemTreeDock,
      datasetViewDock);

  restoreState();
}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("CloudView");
}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);

  if( fileSystemTreeDock ) {
    settings.setValue("fileSystemTree/absoluteFilePath",
        fileSystemTreeDock->currentAbsoluteFilePath());
  }
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->displayPath(settings.value(
        "fileSystemTree/absoluteFilePath").toString());
  }
}


void MainWindow::setupMainMenu()
{
  Base::setupMainMenu();

//  //
//  // File
//  //
//
//  menuFile_->addAction(reloadCurrentFileAction =
//      createAction(getIcon(ICON_reload),
//          "Reload",
//          "Reload current file from disk (Ctrl+R)",
//          [this]() {
//            QWidget * w = centralStackedWidget->currentWidget();
//            if ( w == imageView ) {
//              // imageView->openImage(imageView->currentFileName());
//            }
//            else if ( w == textView ) {
//              textView->showTextFile(textView->currentFileName());
//            }
//            else if ( w == thumbnailsView ) {
//              thumbnailsView->reload();
//            }
//          },
//          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_R),
//              this, nullptr, nullptr,
//              Qt::WindowShortcut)));
//
//  menuFile_->addAction(selectPreviousFileAction_ =
//      createAction(getIcon(ICON_prev),
//          "Previous (Ctrl+PgUp)",
//          "Select previous file (Ctrl+PgUp)",
//          [this]() {
//            thumbnailsView->selectPrevIcon();
//          },
//          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp),
//              this, nullptr, nullptr,
//              Qt::WindowShortcut)));
//
//  menuFile_->addAction(selectNextFileAction =
//      createAction(getIcon(ICON_next),
//          "Next (Ctrl+PgDown)",
//          "Select next file (Ctrl+PgDown)",
//          [this]() {
//            thumbnailsView->selectNextIcon();
//          },
//          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown),
//              this, nullptr, nullptr,
//              Qt::WindowShortcut)));
//
//  menuFile_->addSeparator();
//
//  saveImageAsAction =
//      menuFile_->addAction("Save current image as...",
//          this, &ThisClass::onSaveCurrentImageAs);
//
//  saveImageAsAction->setEnabled(is_visible(imageView) &&
//      !imageView->currentImage().empty());
//
//  saveDisplayImageAsAction =
//      menuFile_->addAction("Save current display image as...",
//          this, &ThisClass::onSaveCurrentDisplayImageAs);
//
//  saveDisplayImageAsAction->setEnabled(is_visible(imageView) &&
//      !imageView->displayImage().empty());
//
//  saveImageMaskAction =
//      menuFile_->addAction("Save current image mask...",
//          this, &ThisClass::onSaveCurrentImageMask);
//
//  saveImageMaskAction->setEnabled(is_visible(imageView) &&
//      !imageView->currentMask().empty());
//
//  loadImageMaskAction =
//      menuFile_->addAction("Set current image mask from file...",
//          this, &ThisClass::onLoadCurrentImageMask);
//
//  loadImageMaskAction->setEnabled(is_visible(imageView) &&
//      !imageView->currentImage().empty());
//
  menuFile_->addSeparator();

//  loadStackAction =
//      menuFile_->addAction("Load stack config...",
//          this, &ThisClass::onLoadStackConfig);
//

  menuFile_->addSeparator();

  quitAppAction =
      menuFile_->addAction("Quit",
          this, &ThisClass::close);


  //
  // Edit
  //

//  menuEdit_->addAction(copyDisplayImageAction =
//      createAction(QIcon(),
//          "Copy display image to clipboard (Ctrl+c)",
//          "Copy display image to clipboard (Ctrl+c)",
//          [this]() {
//
//            if ( is_visible(cloudView) ) {
//              cloudView->copyViewportToClipboard();
//            }
//            else if ( is_visible(imageView) ) {
//              if ( imageView->roiShape()->isVisible() ) {
//                imageView->copyDisplayImageROIToClipboard(imageView->roiShape()->iSceneRect());
//              }
//              else {
//                imageView->copyDisplayImageToClipboard();
//              }
//            }
//          },
//          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
//              this, nullptr, nullptr,
//              Qt::WindowShortcut)));

//  menuEdit_->addAction(copyDisplayViewportAction =
//      createAction(QIcon(),
//          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
//          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
//          [this]() {
//            if ( is_visible(cloudView) ) {
//              cloudView->copyViewportToClipboard();
//            }
//            else if ( is_visible(imageView) ) {
//              QPixmap pxmap = imageView->sceneView()->grab();
//              if ( !pxmap.isNull() ) {
//                QApplication::clipboard()->setPixmap(pxmap);
//              }
//            }
//          },
//          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
//              this, nullptr, nullptr,
//              Qt::WindowShortcut)));


  //
  // View
  //
//  menuView_->addAction(viewInputOptionsAction =
//      createCheckableAction(QIcon(),
//          "Input Options...",
//          "Configure general input options",
//          is_visible(inputOptionsDlgBox),
//          this,
//          &ThisClass::onViewInputOptions));
//
//
//  pipelineProgressView = new QPipelineProgressView(this);
//  menuBar()->setCornerWidget(pipelineProgressView, Qt::TopRightCorner );
//  pipelineProgressView->hide();
//
//  connect(pipelineProgressView, &QPipelineProgressView::progressTextChanged,
//      this, &ThisClass::onStackProgressViewTextChanged,
//      Qt::QueuedConnection);

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

        centralStackedWidget->setCurrentWidget(thumbnailsView);
        thumbnailsView->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::directoryItemPressed,
      [this](const QString & abspath) {

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



void MainWindow::setupThumbnailsView()
{
  connect(thumbnailsView, &QThumbnailsView::showInDirTreeRequested,
      [this](const QString & abspath) {

        if ( fileSystemTreeDock ) {
          fileSystemTreeDock->show(),
          fileSystemTreeDock->raise(),
          fileSystemTreeDock->displayPath(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::currentIconChanged,
      [this](const QString & abspath) {
        if ( !is_visible(thumbnailsView) ) {
          // openImage(abspath);
        }
      });

    //  connect(thumbnailsView, &QThumbnailsView::iconDoubleClicked,
    //      this, &ThisClass::openImage);
    //
    //  connect(thumbnailsView, &QThumbnailsView::iconEnterPressed,
    //      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::customContextMenuRequested,
      this, &ThisClass::onThumbnailsViewCustomContextMenuRequested);

}


void MainWindow::setupDatasetView()
{
  datasetViewDock =
      addCloudViewDatasetViewDock(this, Qt::LeftDockWidgetArea,
          "datasetViewDock",
          "Datasets",
          menuView_);


  datasetView = datasetViewDock->datasetView();
}



///////////////////////////////////////////////////////////////////////////////
}  // namespace cloudview
