/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/QToolbarSpacer.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/qstackingthread/QStackingThread.h>
#include <core/debug.h>

namespace qserstacker {
///////////////////////////////////////////////////////////////////////////////


#define ICON_reload       "reload"
#define ICON_prev         "prev"
#define ICON_next         "next"
#define ICON_like         "like"
#define ICON_dislike      "dislike"
#define ICON_close        "close"
#define ICON_histogram    "histogram"
#define ICON_marker_blue  "marker-blue"

#define ICON_copy         "copy"
#define ICON_delete       "delete"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/gui/icons/%1").arg(name));
}

static bool isTextFile(const QString & abspath)
{
  const QString suffix = QFileInfo(abspath).suffix();

  static const char * textfiles[] = {
      "txt", "doc", "xml", "md"
  };

  for (  uint i = 0; i < sizeof(textfiles)/sizeof(textfiles[0]); ++i ) {
    if ( suffix.compare(textfiles[i], Qt::CaseInsensitive) == 0 ) {
      return true;
    }
  }

  return false;
}

MainWindow::MainWindow()
{

  static const auto createScrollableWrap =
      [](QWidget * w, QWidget * parent = Q_NULLPTR) -> QScrollArea *
  {
    QScrollArea * scrollArea = new QScrollArea(parent ? parent : w->parentWidget());
    scrollArea->setWidgetResizable(true);
    scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(w);
    return scrollArea;
  };


  setWindowIcon(QIcon(":/icons/jup.png"));
  updateWindowTittle();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));
  centralStackedWidget->addWidget(imageEditor = new QImageFileEditor(this));
  centralStackedWidget->addWidget(textViewer = new QTextFileViewer(this));
  centralStackedWidget->addWidget(stackOptionsView = new QStackOptions(this));


  imageEditor->setDisplayFunction(QImageViewer::DisplayFunction(
      [this] (const cv::Mat & src, cv::Mat & dst, int ddepth) -> void {
        image_display_function_(src, dst, ddepth);
      }));



  ///////////////////////////////////
  // Setup main menu items

  menuBar()->setNativeMenuBar(false);
  fileMenu = menuBar()->addMenu("&File");
  viewMenu = menuBar()->addMenu("&View");


  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );

//
//
//  logWidgetDock = addCustomDock(this, Qt::BottomDockWidgetArea,
//      "logWidgetDock",
//      "Debug Log",
//      logWidget = new QLogWidget(this),
//      viewMenu);
//
//  logWidget->startLogging();
//
//
  fileSystemTreeDock = addFileSystemTreeDock(this, Qt::LeftDockWidgetArea,
      "fileSystemTreeDock",
      "Directory Tree",
      viewMenu);


  stackTreeDock = addSequencesTreeDock(this, Qt::LeftDockWidgetArea,
      "sequencesTreeDock",
      "Sequences",
      viewMenu);

  stackTreeView =
      stackTreeDock->sequencesView();

  stackTreeView->
      set_stacklist(stacklist_);


  imageProcessorSelectorDock = addCustomDock(this,
      Qt::LeftDockWidgetArea,
      "imageProcessorSettingsDock",
      "Image Processing",
      createScrollableWrap(imageProcessorSelector = new QImageProcessorSelector(this)),
      viewMenu);

  tabifyDockWidget(fileSystemTreeDock, stackTreeDock);
  tabifyDockWidget(stackTreeDock, imageProcessorSelectorDock);
  fileSystemTreeDock->raise();


//  imageProcessorSettings->set_processor(stackProgressView->
//      currentImageProcessor());

  ///////////////////////////////////
  // Configure events

  connect(fileSystemTreeDock, &QFileSystemTreeDock::currentDirectoryChanged,
      [this](const QString & abspath) {

        if ( stackProgressView ) {
          stackProgressView->setImageViewer(nullptr);
        }

        imageEditor->clear();
        textViewer->clear();
        centralStackedWidget->setCurrentWidget(thumbnailsView);
        thumbnailsView->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::directoryItemPressed,
      [this](const QString & abspath) {

        if ( stackProgressView ) {
          stackProgressView->setImageViewer(nullptr);
        }
        imageEditor->clear();
        textViewer->clear();

        if ( centralStackedWidget->currentWidget() != thumbnailsView ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
          if ( thumbnailsView->currentPath() != abspath ) {
            thumbnailsView->displayPath(abspath);
          }
        }
      });

  connect(thumbnailsView, &QThumbnailsView::showInDirTreeRequested,
      [this](const QString & abspath) {
        if ( stackProgressView ) {
          stackProgressView->setImageViewer(nullptr);
        }
        imageEditor->clear();
        textViewer->clear();
        fileSystemTreeDock->show(),
        fileSystemTreeDock->raise(),
        fileSystemTreeDock->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::customContextMenuRequested,
      this, &ThisClass::onFileSystemTreeCustomContextMenuRequested);

  connect(thumbnailsView, &QThumbnailsView::currentIconChanged,
      [this](const QString & abspath) {

        if ( stackProgressView ) {
          stackProgressView->setImageViewer(nullptr);
        }

        imageEditor->clear();
        textViewer->clear();

        if ( imageEditor->isVisible() || textViewer->isVisible() ) {
          openImage(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::iconDoubleClicked,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::iconEnterPressed,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::customContextMenuRequested,
      this, &ThisClass::onThumbnailsViewCustomContextMenuRequested);

  connect(stackTreeView, &QStackTree::currentItemChanged,
      this, &ThisClass::onStackTreeCurrentItemChanged);

  connect(stackTreeView, &QStackTree::itemDoubleClicked,
      this, &ThisClass::onStackTreeItemDoubleClicked);

  connect(stackTreeView, &QStackTree::showStackOptionsClicked,
      this, &ThisClass::onShowStackOptionsClicked);

  connect(stackOptionsView, &QStackOptions::closeWindowRequested,
      [this]() {
        if ( !QStackingThread::isRunning() ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
        }
        else {
          centralStackedWidget->setCurrentWidget(imageEditor);
          stackProgressView->setImageViewer(imageEditor);
        }
      });


  connect(stackOptionsView, &QStackOptions::applyInputOptionsToAllRequested,
      stackTreeView, & QStackTree::applyInputOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyMasterFrameOptionsToAllRequested,
      stackTreeView, & QStackTree::applyMasterFrameOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyROISelectionOptionsToAllRequested,
      stackTreeView, &QStackTree::applyROISelectionOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyFrameAccumulationOptionsToAllRequested,
      stackTreeView, & QStackTree::applyFrameAccumulationOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyFrameRegistrationOptionsToAllRequested,
      stackTreeView, & QStackTree::applyFrameRegistrationOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyOutputOptionsToAllRequested,
      stackTreeView, & QStackTree::applyOutputOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyAllStackOptionsToAllRequested,
      stackTreeView, & QStackTree::applyAllStackOptionsToAll);

  connect(stackOptionsView, &QStackOptions::stackNameChanged,
      stackTreeView, &QStackTree::updateStackName);

  connect(stackTreeView, &QStackTree::stackNameChanged,
      [this] (const c_image_stacking_options::ptr & options) {
        if ( stackOptionsView->currentStack() == options ) {
          stackOptionsView->updateControls();
        }});

  connect(QStackingThread::singleton(), &QStackingThread::started,
      this, &ThisClass::onStackingThreadStarted);

  connect(QStackingThread::singleton(), &QStackingThread::finished,
      this, &ThisClass::onStackingThreadFinished);

  connect(imageProcessorSelector, &QImageProcessorSelector::parameterChanged,
      [this]() {
        imageEditor->set_current_processor(imageProcessorSelector->current_processor());
      });

//  connect(imageProcessorSelector, &QImageProcessorSelector::currentImageProcessorChanged,
//      [this]() {
//        imageEditor->set_current_processor(imageProcessorSelector->current_processor());
//      });
//
//  connect(imageProcessorSelector, &QImageProcessorSelector::imageProcessingEnableChanged,
//      [this](bool) {
//        imageEditor->set_current_processor(imageProcessorSelector->current_processor());
//      });

//  connect(pipelinesTreeView_ctl, &QStackListTree::currentItemChanged,
//      this, &ThisClass::onPipelineTreeViewCurrentItemChanged);
//  connect(pipelinesTreeView_ctl, &QStackListTree::pipelineItemPressed,
//      this, &ThisClass::onPipelineItemPressed);
//  connect(pipelinesTreeView_ctl, &QStackListTree::pipelineItemClicked,
//      this, &ThisClass::onPipelineItemClicked);
//  connect(pipelinesTreeView_ctl, &QStackListTree::pipelineItemDoubleClicked,
//      this, &ThisClass::onPipelineItemDoubleClicked);
//  connect(pipelinesTreeView_ctl, &QStackListTree::pipelineItemActivated,
//      this, &ThisClass::onPipelineItemActivated);
//  connect(pipelinesTreeView_ctl, &QStackListTree::pipelineItemEntered,
//      this, &ThisClass::onPipelineItemEntered);


  ///////////////////////////////////
  QAction * action;
//
//  fileMenu->addAction(action = new QAction("Add stack..."));
//  connect(action, &QAction::triggered,
//      this, &ThisClass::onAddStack);
//

  fileMenu->addSeparator();
  fileMenu->addAction(action = new QAction("Quit"));
  connect(action, &QAction::triggered, []() {
    QApplication::quit();
  });


  ///////////////////////////////////

  restoreGeometry();
  restoreState();
  configureImageViewerToolbars();
  configureTextViewerToolbars();


  image_processors_->load(c_image_processor_collection::default_processor_collection_path());
  imageProcessorSelector->set_available_processors(image_processors_);
  imageEditor->set_current_processor(imageProcessorSelector->current_processor());

}

MainWindow::~MainWindow()
{
  saveState();
  saveGeometry();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("QSkyStacker");
}

void MainWindow::saveGeometry()
{
  QSettings settings;
  settings.setValue("MainWindow/Geometry", Base::saveGeometry());
  settings.setValue("MainWindow/State", Base::saveState());
}

void MainWindow::restoreGeometry()
{
  QSettings settings;
  Base::restoreGeometry(settings.value("MainWindow/Geometry").toByteArray());
  Base::restoreState(settings.value("MainWindow/State").toByteArray());
}

void MainWindow::saveState()
{
  QSettings settings;
  settings.setValue("fileSystemTree/absoluteFilePath",
      fileSystemTreeDock->currentAbsoluteFilePath());
}

void MainWindow::restoreState()
{
  QSettings settings;
  fileSystemTreeDock->displayPath(settings.value(
      "fileSystemTree/absoluteFilePath").toString());
}

void MainWindow::configureTextViewerToolbars()
{
  QToolBar * toolbar;
  QAction * action;
  QLabel * imageNameLabel;
  QLabel * imageSizeLabel;
  QShortcut * shortcut;

  toolbar = textViewer->toolbar();

  toolbar->addAction(action = new QAction(getIcon(ICON_prev), "Previous"));
  action->setToolTip("Load previous image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectPrevIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_next), "Next"));
  action->setToolTip("Load next image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectNextIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_reload), "Reload"));
  action->setToolTip("Reaload current image from disk");
  connect(action, &QAction::triggered, [this]() {
    textViewer->showTextFile(textViewer->currentFileName());
  });


  toolbar->addSeparator();


  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  const auto onTextViewerCurrentSourceChanged =
      [this, imageNameLabel] () {

    const QString abspath = textViewer->currentFileName();
    imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());

  };

  connect(textViewer, &QTextFileViewer::currentFileNameChanged,
      onTextViewerCurrentSourceChanged);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());


  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    textViewer->clear();
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });

}


void MainWindow::configureImageViewerToolbars()
{
  QToolBar * toolbar;
  QStatusBar * statusbar;
  QAction * action;
  QLabel * imageNameLabel;
  QLabel * imageSizeLabel;
  QShortcut * shortcut;
//  QScaleSelectionButton * scaleSelectionCtl;

  toolbar = imageEditor->embedToolbar();
  statusbar = imageEditor->embedStatusbar();


  toolbar->addAction(action = new QAction(getIcon(ICON_prev), "Previous"));
  action->setToolTip("Load previous image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectPrevIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_next), "Next"));
  action->setToolTip("Load next image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectNextIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_reload), "Reload"));
  action->setToolTip("Reaload current image from disk");
  connect(action, &QAction::triggered, [this]() {
    imageEditor->openImage(imageEditor->currentFileName());
  });


  toolbar->addSeparator();


  toolbar->addAction(action = new QAction(getIcon(ICON_dislike), "Bad"));
  action->setToolTip("Move current image to the .bads subfolder (Ctrl+DEL)");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Delete));
  connect(action, &QAction::triggered, [this]() {
    if ( imageEditor->isVisible() ) {
      thumbnailsView->moveToBads(imageEditor->currentFileName());
    }
  });


  toolbar->addSeparator();



  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  toolbar->addWidget(imageSizeLabel = new QLabel(""));
  imageSizeLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  const auto onImageEditorCurrentImageChanged =
      [this, imageNameLabel, imageSizeLabel] () {

    const QString abspath = imageEditor->currentFileName();
    imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
    imageSizeLabel->setText(QString("%1x%2").arg(imageEditor->image().cols).arg(imageEditor->image().rows));

  };

  connect(imageEditor, &QImageFileEditor::currentImageChanged,
      onImageEditorCurrentImageChanged);

  toolbar->addSeparator();




  toolbar->addWidget(new QToolbarSpacer());

//  toolbar->addWidget(scaleSelectionCtl = new QScaleSelectionButton());
//  connect(scaleSelectionCtl, &QScaleSelectionButton::scaleChanged, [this](int v) {
//    imageView->setScale(v);
//  });


  toolbar->addAction(action = new QAction(getIcon(ICON_marker_blue), "Marker"));
  action->setToolTip("Show rectange marker.\nUse Ctrl+M to move the marker into center of view.");
  action->setCheckable(true);
  action->setChecked(imageEditor->selectionRectIsVisible());
  shortcut = new QShortcut(QKeySequence("Ctrl+M"), toolbar);
  connect(shortcut, &QShortcut::activated, [this, action]() {
    if ( imageEditor->isVisible() ) {
      if ( imageEditor->selectionRectIsVisible() ) {
        imageEditor->setSelectionRectVisible(false);
      }
      else {
        imageEditor->selectionRectToCenterOfView();
      }
      action->setChecked(imageEditor->selectionRectIsVisible());
    }
  });
  connect(action, &QAction::triggered, [this](bool checked) {
    imageEditor->setSelectionRectVisible(checked);
  });





  toolbar->addAction(action = new QAction(getIcon(ICON_histogram), "Levels"));
  action->setToolTip("Configure Display Image Levels");
  action->setCheckable(true);
  action->setChecked(false);
  //action->setShortcut(QKeySequence::Cancel);
  connect(action, &QAction::triggered, [this, action](bool checked) {

    if ( checked && !imageLevelsDialogBox ) {

      imageLevelsDialogBox = new QMtfControlDialogBox(this);
      imageLevelsDialogBox->setMtf(image_display_function_.mtf());

      connect(imageLevelsDialogBox, &QMtfControlDialogBox::mtfChanged,
          [this]() {
            if ( imageEditor->isVisible() ) {
              QWaitCursor wait(this);
              imageEditor->updateDisplay();
            }
          });

      connect(imageLevelsDialogBox, &QMtfControlDialogBox::visibilityChanged,
          action, &QAction::setChecked);
    }

    if ( imageLevelsDialogBox ) {
      if ( !checked ) {
        imageLevelsDialogBox->hide();
      }
      else {
        imageLevelsDialogBox->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
        imageLevelsDialogBox->setImage(imageEditor->image());
        imageLevelsDialogBox->showNormal();
      }
    }

  });

  connect(imageEditor, &QImageFileEditor::currentImageChanged,
      [this] () {
        if ( imageLevelsDialogBox && imageLevelsDialogBox->isVisible() ) {
          imageLevelsDialogBox->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
          imageLevelsDialogBox->setImage(imageEditor->image());
        }
      });



  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    imageEditor->closeCurrentSequence();
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });

  connect(imageEditor, &QImageFileEditor::onMouseMove,
      [this, statusbar](QMouseEvent * e) {
        statusbar->showMessage(imageEditor->statusStringForPixel(e->pos()));
    });

}


void MainWindow::openImage(const QString & abspath)
{
  if ( stackProgressView ) {
    stackProgressView->setImageViewer(nullptr);
  }

  imageEditor->clear();
  textViewer->clear();

  if ( isTextFile(abspath) ) {
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
  }
  else {
    centralStackedWidget->setCurrentWidget(imageEditor);
    imageEditor->openImage(abspath);
  }
}



void MainWindow::onFileSystemTreeCustomContextMenuRequested(const QPoint & pos,
    const QFileInfoList & selectedItems )
{
  QMenu menu;
  QAction * act;
  QString path, name;


  fileSystemTreeDock->fillContextMenu(menu, selectedItems);

  menu.addSeparator();

//  QFileInfo curretPath(path = fileSystemTreeDock->currentAbsoluteFilePath());
//  if ( curretPath.isDir() && !wkspace->batches((name = curretPath.fileName()).toStdString()) ) {
//    menu.addAction(act = new QAction("Create batch here..."));
//    connect(act, &QAction::triggered, [this, path, name]() {
//      this->addBatch(path, name);
//    });
//  }

  if ( !menu.isEmpty() ) {
    menu.exec(pos);
  }
}



void MainWindow::onThumbnailsViewCustomContextMenuRequested(const QPoint &pos)
{
  QMenu poupupMenu;
  thumbnailsView->populateContextMenu(&poupupMenu, pos);
  if ( !poupupMenu.isEmpty() ) {
    poupupMenu.exec(thumbnailsView->contextMenuPosToGlobal(pos));
  }
}

void MainWindow::onStackTreeCurrentItemChanged(const c_image_stacking_options::ptr & selectedStack,
    const c_input_source::ptr & selectedInputSource)
{
  if ( QStackingThread::isRunning() ) {

    //QStackingThread::auto_lock lock;

    if ( selectedInputSource ) {
      stackProgressView->setImageViewer(nullptr);
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(selectedInputSource->filename());
    }
    else if ( selectedStack ) {

      if ( selectedStack == QStackingThread::currentStack() ) {
        stackProgressView->setImageViewer(imageEditor);
      }
      else {

        stackProgressView->setImageViewer(nullptr);

        QWidget * currentCentralWidget =
            centralStackedWidget->currentWidget();

        if ( currentCentralWidget == stackOptionsView ) {
          stackOptionsView->setCurrentStack(selectedStack);
        }
        else if ( thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false) ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
        }
      }
    }

  }
  else if ( selectedInputSource ) {
    centralStackedWidget->setCurrentWidget(imageEditor);
    imageEditor->openImage(selectedInputSource->filename());
  }
  else if ( selectedStack ) {
    if ( centralStackedWidget->currentWidget() == thumbnailsView ) {
      thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false);
    }
    else {
      stackOptionsView->setCurrentStack(selectedStack);
      centralStackedWidget->setCurrentWidget(stackOptionsView);
    }
  }

}

void MainWindow::onStackTreeItemDoubleClicked(const c_image_stacking_options::ptr & selectedStack,
    const c_input_source::ptr & selectedInputSource)
{
  if ( QStackingThread::isRunning() ) {

    if ( selectedInputSource ) {
      stackProgressView->setImageViewer(nullptr);
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(selectedInputSource->filename());
    }
    else if ( selectedStack ) {

      QWidget * currentCentralWidget =
          centralStackedWidget->currentWidget();

      if ( selectedStack == QStackingThread::currentStack() ) {

        if ( currentCentralWidget == imageEditor ) {

          if ( !stackProgressView->imageViewer() ) {
            stackProgressView->setImageViewer(imageEditor);
          }
          else {
            stackProgressView->setImageViewer(nullptr);
            stackOptionsView->setCurrentStack(selectedStack);
            centralStackedWidget->setCurrentWidget(stackOptionsView);
          }

        }
        else if ( currentCentralWidget == stackOptionsView ) {
          if ( thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
          else {
            centralStackedWidget->setCurrentWidget(imageEditor);
            stackProgressView->setImageViewer(imageEditor);
          }
        }
        else {
          centralStackedWidget->setCurrentWidget(imageEditor);
          stackProgressView->setImageViewer(imageEditor);
        }
      }
      else {

        if ( currentCentralWidget == stackOptionsView ) {
          if ( thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
        }
        else {
          stackOptionsView->setCurrentStack(selectedStack);
          centralStackedWidget->setCurrentWidget(stackOptionsView);
        }
      }
    }

  }
  else if ( selectedInputSource ) {
    if ( centralStackedWidget->currentWidget() == imageEditor ) {
      centralStackedWidget->setCurrentWidget(thumbnailsView);
      thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false);
    }
    else {
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(selectedInputSource->filename());
    }
  }
  else if ( selectedStack ) {
    if ( centralStackedWidget->currentWidget() == stackOptionsView ) {
      if ( thumbnailsView->setCurrentPath(selectedStack->get_displaypatch().c_str(), false) ) {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }
    }
    else {
      stackOptionsView->setCurrentStack(selectedStack);
      centralStackedWidget->setCurrentWidget(stackOptionsView);
    }
  }

}


void MainWindow::onShowStackOptionsClicked(const c_image_stacking_options::ptr & stack)
{
  if ( stack ) {

    if ( stackProgressView ) {
      stackProgressView->setImageViewer(nullptr);
    }

    stackOptionsView->setCurrentStack(stack);
    centralStackedWidget->setCurrentWidget(stackOptionsView);
  }

}

void MainWindow::onStackingThreadStarted()
{
  if ( !stackProgressView ) {
    stackProgressView = new QStackingProgressView(this);
  }

  if ( centralStackedWidget->currentWidget() != imageEditor ) {
    centralStackedWidget->setCurrentWidget(imageEditor);
  }

  imageEditor->clear();
  stackProgressView->setImageViewer(imageEditor);

  if ( !stackProgressView->isVisible() ) {
    stackProgressView->show();
  }
}

void MainWindow::onStackingThreadFinished()
{
  if( stackProgressView ) {

    // it may be that there is next task in queue,
    // don't blink with this dialog box
    QTimer::singleShot(500,
        [this]() {
          if ( stackProgressView->isVisible() && !QStackingThread::isRunning() ) {
            stackProgressView->hide();
          }
        });

  }
}


///////////////////////////////////////////////////////////////////////////////
}  // namespace qskystacker
