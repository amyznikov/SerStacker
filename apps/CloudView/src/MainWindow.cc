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


#define ICON_roi                ":/cloudview/icons/roi.png"
#define ICON_point_size         ":/cloudview/icons/degree.png"
#define ICON_brightness         ":/cloudview/icons/brightness.png"
#define ICON_bgcolor            ":/cloudview/icons/fill-color.png"
#define ICON_cloud_rotate       ":/cloudview/icons/cloud_rotate.png"
#define ICON_cloud_view_target  ":/cloudview/icons/cloud_view_target.png"


namespace cloudview {
///////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/cloudview/icons/app-icon.png"));
  updateWindowTittle();
  QImageProcessorsCollection::load();

  setCentralWidget(centralStackedWidget =
      new QStackedWidget(this));

  centralStackedWidget->addWidget(thumbnailsView =
      new QThumbnailsView(this));

  centralStackedWidget->addWidget(inputSourceView =
      new QInputSourceView(this));

  imageView = inputSourceView->imageView();
  cloudView = inputSourceView->cloudView();
  textView = inputSourceView->textView();



  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  setupMainMenu();
  setupLogWidget();
  setupFileSystemTreeView();
  setupThumbnailsView();
  setupMtfControls();
  setupStatusbar();
  setupInputSourceView();

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

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

  sb->addWidget(statusbarShapesLabel_ctl = new QLabel(this));
  sb->addWidget(statusbarMousePosLabel_ctl = new QLabel(this));
  sb->addPermanentWidget(statusbarShowLog_ctl = new QToolButton());
  statusbarShowLog_ctl->setDefaultAction(showLogWidgetAction_);
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


void MainWindow::onShowCloudViewSettingsDialogBoxActionClicked(bool checked)
{
  if ( checked && !cloudViewSettingsDialogBox ) {

    cloudViewSettingsDialogBox = new QPointCloudViewSettingsDialogBox(this);
    cloudViewSettingsDialogBox->setCloudViewer(cloudView);
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

void MainWindow::setupInputSourceView()
{
  QToolBar * toolbar;

  ///////////////////////////////////////////////////////////////////////
  toolbar = inputSourceView->toolbar();

  ///////////////////////////////////////////////////////////////////////
  toolbar = inputSourceView->imageViewToolbar();

  toolbar->addAction(showMtfControlAction_);

  ///////////////////////////////////////////////////////////////////////
  toolbar = inputSourceView->cloudViewToolbar();

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
  toolbar = inputSourceView->textViewToolbar();

  ///////////////////////////////////////////////////////////////////////
  toolbar = inputSourceView->rightToolbar();

  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          inputSourceView, nullptr, nullptr,
          Qt::WindowShortcut)));


}


void MainWindow::openImage(const QString & abspath)
{
  QWaitCursor wait(this);

  centralStackedWidget->setCurrentWidget(inputSourceView);
  inputSourceView->openFile(abspath);
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  if( !visible ) {
    mtfControl_->setMtfDisplaySettings(nullptr);
  }
  else if ( is_visible(inputSourceView) ) {
    mtfControl_->setMtfDisplaySettings(inputSourceView->mtfDisplay());
  }
  else {
    mtfControl_->setMtfDisplaySettings(nullptr);
  }

  const QString currentFileName =
      mtfControl_->mtfDisplaySettings() ?
          QFileInfo(inputSourceView->currentFileName()).fileName() :
          "";

  if( currentFileName.isEmpty() ) {
    mtfControl_->setWindowTitle("Adjust Display Levels ...");
  }
  else {
    mtfControl_->setWindowTitle(qsprintf("Adjust Display Levels: %s",
        currentFileName.toUtf8().constData()));
  }
}




///////////////////////////////////////////////////////////////////////////////
}  // namespace cloudview
