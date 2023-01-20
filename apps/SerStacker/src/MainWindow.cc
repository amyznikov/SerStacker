/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/QToolbarSpacer.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qstackingthread/QStackingThread.h>
#include <gui/qgraphicsshape/QShapesButton.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qimagesave/QImageSaveOptions.h>
#include <gui/qthumbnailsview/QThumbnails.h>
#include <core/io/load_image.h>
#include <core/debug.h>

namespace qserstacker {
///////////////////////////////////////////////////////////////////////////////


#define ICON_reload       ":/gui/icons/reload"
#define ICON_prev         ":/gui/icons/prev"
#define ICON_next         ":/gui/icons/next"
#define ICON_like         ":/gui/icons/like"
#define ICON_dislike      ":/gui/icons/dislike"
#define ICON_close        ":/gui/icons/close"
#define ICON_histogram    ":/gui/icons/histogram"
#define ICON_marker_blue  ":/gui/icons/marker-blue"
#define ICON_reference    ":/gui/icons/reference"
#define ICON_options      ":/gui/icons/options"
#define ICON_mask         ":/gui/icons/mask"
#define ICON_frame        ":/gui/icons/frame"
#define ICON_badframe     ":/gui/icons/badframe"

#define ICON_copy         ":/gui/icons/copy"
#define ICON_delete       ":/gui/icons/delete"

#define ICON_roi          ":/icons/roi.png"
#define ICON_metrics      ":/icons/metrics.png"


namespace  {

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


}  // namespace


MainWindow::MainWindow()
{

  static const auto createScrollableWrap =
      [](QWidget * w, QWidget * parent = nullptr) -> QScrollArea *
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
  centralStackedWidget->addWidget(imageEditor = new QImageEditor(this));
  centralStackedWidget->addWidget(textViewer = new QTextFileViewer(this));
  centralStackedWidget->addWidget(stackOptionsView = new QStackOptions(this));

#if HAVE_QGLViewer
  centralStackedWidget->addWidget(cloudViewer = new QCloudViewer(this));
  connect(centralStackedWidget, &QStackedWidget::currentChanged,
      [this]() {
        if ( cloudViewSettingsDialogBox && cloudViewSettingsDialogBox->isVisible() ) {
          if ( !cloudViewer->isVisible() ) {
            cloudViewSettingsDialogBox->hide();
          }
        }
      });
#endif // HAVE_QGLViewer



  ///////////////////////////////////
  // Setup main menu items

  menuBar()->setNativeMenuBar(false);
  fileMenu = menuBar()->addMenu("&File");
  editMenu = menuBar()->addMenu("&Edit");
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

  imageProcessorSelectorDock = addCustomDock(this,
      Qt::LeftDockWidgetArea,
      "imageProcessorSettingsDock",
      "Image Processing",
      imageProcessorSelector = new QImageProcessorSelector(this), //createScrollableWrap(imageProcessorSelector = new QImageProcessorSelector(this)),
      viewMenu);

  tabifyDockWidget(fileSystemTreeDock, stackTreeDock);
  tabifyDockWidget(stackTreeDock, imageProcessorSelectorDock);
  fileSystemTreeDock->raise();


  ///////////////////////////////////
  // Configure events

  connect(fileSystemTreeDock, &QFileSystemTreeDock::currentDirectoryChanged,
      [this](const QString & abspath) {

        if ( stackProgressView ) {
          stackProgressView->setImageViewer(nullptr);
        }

        imageEditor->clear();
        textViewer->clear();
#if HAVE_QGLViewer
        cloudViewer->clear();
#endif
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
#if HAVE_QGLViewer
        cloudViewer->clear();
#endif

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
#if HAVE_QGLViewer
        cloudViewer->clear();
#endif
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
#if HAVE_QGLViewer
        cloudViewer->clear();
#endif
        if ( imageEditor->isVisible() ) {
          openImage(abspath);
        }
        else if ( textViewer->isVisible() ) {
          openImage(abspath);
        }
#if HAVE_QGLViewer
        else if ( cloudViewer->isVisible() ) {
          openImage(abspath);
        }
#endif
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

  connect(stackOptionsView, &QStackOptions::applyROISelectionOptionsToAllRequested,
      stackTreeView, &QStackTree::applyROISelectionOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyFrameUpscaleOptionsToAllRequested,
      stackTreeView, &QStackTree::applyFrameUpscaleOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyFrameRegistrationOptionsToAllRequested,
      stackTreeView, & QStackTree::applyFrameRegistrationOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyFrameAccumulationOptionsToAllRequested,
      stackTreeView, & QStackTree::applyFrameAccumulationOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyOutputOptionsToAllRequested,
      stackTreeView, & QStackTree::applyOutputOptionsToAll);

  connect(stackOptionsView, &QStackOptions::applyAllStackOptionsToAllRequested,
      stackTreeView, & QStackTree::applyAllStackOptionsToAll);

  connect(stackOptionsView, &QStackOptions::stackNameChanged,
      [this](const c_image_stacking_options::ptr & stack) {
        stackTreeView->updateStackName(stack);
        saveCurrentWork();
    });

  connect(stackOptionsView, &QStackOptions::stackOptionsChanged,
      this, &ThisClass::saveCurrentWork );

  connect(stackTreeView, &QStackTree::stackNameChanged,
      [this] (const c_image_stacking_options::ptr & stack) {
        if ( stackOptionsView->currentStack() == stack ) {
          stackOptionsView->updateControls();
        }
        saveCurrentWork();
      });

  connect(stackTreeView, &QStackTree::stackCollectionChanged,
        this, &ThisClass::saveCurrentWork );

  connect(stackTreeView, &QStackTree::stackSourcesChanged,
      this, &ThisClass::saveCurrentWork );
//
//  connect(stackTreeView, &QStackTree::stackDeleted,
//      this, &ThisClass::saveCurrentWork );

  connect(QStackingThread::singleton(), &QStackingThread::started,
      this, &ThisClass::onStackingThreadStarted);

  connect(QStackingThread::singleton(), &QStackingThread::finished,
      this, &ThisClass::onStackingThreadFinished);

  connect(imageProcessorSelector, &QImageProcessorSelector::parameterChanged,
      [this]() {
        imageEditor->set_current_processor(imageProcessorSelector->current_processor());
      });


  ///////////////////////////////////

  QShortcut * shortcut;

  fileMenu->addAction(saveImageAsAction = new QAction("Save current image as..."));
  saveImageAsAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentImage().empty());
  connect(saveImageAsAction, &QAction::triggered, this, &ThisClass::onSaveCurrentImageAs);


  fileMenu->addAction(saveDisplayImageAsAction = new QAction("Save current display image as..."));
  saveDisplayImageAsAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentImage().empty());
  connect(saveDisplayImageAsAction, &QAction::triggered, this, &ThisClass::onSaveCurrentDisplayImageAs);


  fileMenu->addAction(saveImageMaskAction = new QAction("Save current image mask..."));
  saveImageMaskAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentMask().empty());
  connect(saveImageMaskAction, &QAction::triggered, this, &ThisClass::onSaveCurrentImageMask);

  fileMenu->addAction(loadImageMaskAction = new QAction("Set current image mask from file..."));
  loadImageMaskAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentImage().empty());
  connect(loadImageMaskAction, &QAction::triggered, this, &ThisClass::onLoadCurrentImageMask);



  fileMenu->addSeparator();
  fileMenu->addAction(loadStackAction =
      new QAction("Load stack config..."));
  connect(loadStackAction, &QAction::triggered,
      this, &ThisClass::onLoadStackConfig);


  fileMenu->addSeparator();
  fileMenu->addAction(quitAppAction = new QAction("Quit"));
  connect(quitAppAction, &QAction::triggered, []() {
    QApplication::quit();
  });



  imageEditor->addAction(copyDisplayImageAction = new QAction("Copy display image to clipboard", imageEditor));
  editMenu->addAction(copyDisplayImageAction);
  shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), imageEditor);
  connect(shortcut, &QShortcut::activated, copyDisplayImageAction, &QAction::trigger);
  editMenu->addAction(copyDisplayImageAction);


  viewMenu->addAction(viewGeneralSettingsAction = new QAction("General Settings..."));
  viewGeneralSettingsAction->setCheckable(true);
  connect(viewGeneralSettingsAction, &QAction::triggered,
      this, &ThisClass::onViewGeneralSettings);


  connect(copyDisplayImageAction, &QAction::triggered,
      [this]() {
        if ( imageEditor->isVisible() ) {
          imageEditor->copyDisplayImageToClipboard();
        }
      });


  static const auto enableFileMenuActions =
      [this]() {
        saveImageAsAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentImage().empty());
        saveDisplayImageAsAction->setEnabled(imageEditor->isVisible() && !imageEditor->displayImage().empty());
        copyDisplayImageAction->setEnabled(imageEditor->isVisible() && !imageEditor->displayImage().empty());
        saveImageMaskAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentMask().empty());
        loadImageMaskAction->setEnabled(imageEditor->isVisible() && !imageEditor->currentImage().empty());
      };

  connect(imageEditor, &QImageEditor::currentImageChanged,
      enableFileMenuActions);

  connect(imageEditor, &QImageViewer::visibilityChanged,
      enableFileMenuActions);

  ///////////////////////////////////

  configureImageViewerToolbars();
  configureTextViewerToolbars();
  configureCloudViewerToolbars();
  setupFocusGraph();
  setupRoiOptions();
  restoreGeometry();
  restoreState();


  //  image_processors_->load(c_image_processor_collection::default_processor_collection_path());
  //  imageProcessorSelector->set_available_processors(image_processors_);
  imageEditor->set_current_processor(imageProcessorSelector->current_processor());

  stacklist_->load();
  stackTreeView->set_stacklist(stacklist_);

  QApplication::instance()->installEventFilter(this);

}

MainWindow::~MainWindow()
{
  saveState();
  saveGeometry();
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
  if ( event->type() == QEvent::Wheel) {
    const QComboBox * combo = dynamic_cast<const QComboBox*>(watched);
    if ( combo && !combo->isEditable() ) {
      return true;
    }
  }

  return Base::eventFilter(watched, event);
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("SerStacker");
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
  QScaleSelectionButton * scaleSelectionCtl;
  QShapesButton * shapesCtl;

  toolbar = imageEditor->embedToolbar();
  statusbar = imageEditor->embedStatusbar();

  /////////////////////

  toolbar->addAction(action = new QAction(getIcon(ICON_prev), "Previous"));
  action->setToolTip("Load previous image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectPrevIcon();
  });

  /////////////////////

  toolbar->addAction(action = new QAction(getIcon(ICON_next), "Next"));
  action->setToolTip("Load next image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectNextIcon();
  });

  /////////////////////

  toolbar->addAction(action = new QAction(getIcon(ICON_reload), "Reload"));
  action->setToolTip("Reaload current image from disk");
  connect(action, &QAction::triggered, [this]() {
    imageEditor->openImage(imageEditor->currentFileName());
  });

  /////////////////////

  toolbar->addSeparator();

  /////////////////////

  toolbar->addAction(action = new QAction(getIcon(ICON_dislike), "Bad"));
  action->setToolTip("Move current image to the .bads subfolder (Ctrl+DEL)");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Delete));
  connect(action, &QAction::triggered, [this]() {
    if ( imageEditor->isVisible() ) {
      thumbnailsView->moveToBads(imageEditor->currentFileName());
    }
  });


  /////////////////////

  static QIcon badframeIcon;
  badframeIcon.addPixmap(getPixmap(ICON_frame), QIcon::Normal, QIcon::Off);
  badframeIcon.addPixmap(getPixmap(ICON_badframe), QIcon::Normal, QIcon::On);
  toolbar->addAction(badframeAction = new QAction(badframeIcon, "Bad Frame"));
  badframeAction->setCheckable(true);
  badframeAction->setToolTip("Mark/Unmark current frame as bad (Ctrl+A)");
  badframeAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_A));

  connect(badframeAction, &QAction::triggered, [this](bool checked) {
    if ( imageEditor->isVisible() ) {

      const c_input_sequence::ptr & input_sequence =
          imageEditor->input_sequence();

      if ( input_sequence ) {

        c_input_source::ptr source =
            input_sequence->current_source();

        if ( source ) {
          source->set_badframe(input_sequence->current_pos() - 1, checked);
          source->save_badframes();
        }
      }
    }
  });

  connect(imageEditor, &QImageEditor::currentImageChanged,
      [this]() {

        bool checked = false;

        const c_input_sequence::ptr & input_sequence =
            imageEditor->input_sequence();

        if ( input_sequence ) {

          c_input_source::ptr source =
              input_sequence->current_source();

          if ( source ) {
            checked = source->is_badframe(
                input_sequence->current_pos() - 1);
          }
        }

        if ( badframeAction->isChecked() != checked ) {
          badframeAction->setChecked(checked);
        }
      });

  /////////////////////


  toolbar->addAction(setReferenceFrameAction = new QAction(getIcon(ICON_reference), "Make reference"));
  setReferenceFrameAction->setToolTip("Make this frame reference");
  //  setReferenceFrameAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_R));
  connect(setReferenceFrameAction, &QAction::triggered, [this]() {
    if ( imageEditor->isVisible() && stackTreeView->isVisible() ) {

      c_input_source::ptr selectedSource;
      c_image_stacking_options::ptr selectedStack;

      selectedSource = stackTreeView->getCurrentInputSource(&selectedStack);
      if ( selectedSource && selectedStack ) {
        const c_input_sequence::ptr & currentSequence = imageEditor->input_sequence();
        if ( currentSequence && currentSequence->current_source()->filename() == selectedSource->filename() ) {
          selectedStack->master_frame_options().master_source_path = selectedSource->filename();
          selectedStack->master_frame_options().master_frame_index = currentSequence->current_pos() - 1;
        }
      }
    }
  });

  /////////////////////

  toolbar->addSeparator();



  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  /////////////////////

  toolbar->addSeparator();

  toolbar->addWidget(imageSizeLabel = new QLabel(""));
  imageSizeLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  connect(imageEditor, &QImageEditor::currentImageChanged,
      [this, imageNameLabel, imageSizeLabel]() {

        const QString abspath = imageEditor->currentFileName();
        imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
        imageSizeLabel->setText(QString("%1x%2").arg(imageEditor->currentImage().cols).arg(imageEditor->currentImage().rows));

        if ( mtfControl && mtfControl->isVisible() ) {
          if ( imageEditor->currentFileName().isEmpty() ) {
            mtfControl->setWindowTitle("Adjust Display Levels ...");
          }
          else {
            mtfControl->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
          }
        }
      });

  /////////////////////

  toolbar->addSeparator();
  toolbar->addWidget(new QToolbarSpacer());

  /////////////////////

  toolbar->addAction(editMaskAction = new QAction(getIcon(ICON_mask), "mask"));
  editMaskAction->setToolTip("View/edit image mask");
  editMaskAction->setCheckable(true);
  editMaskAction->setChecked(imageEditor->displayType() == QImageViewer::DisplayMask);
  //  editMaskAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_M));
  connect(editMaskAction, &QAction::triggered,
      [this](bool checked) {
        if ( imageEditor->isVisible() ) {

          if ( checked ) {
            createImageViewOptionsControl();
          }
          else if ( imageViewOptionsDlgBox )  {
            delete imageViewOptionsDlgBox;
            imageViewOptionsDlgBox = nullptr;
          }

          //imageEditor->setDisplayType(checked ? QImageViewer::DisplayMask : QImageViewer::DisplayImage);
        }
      });


  /////////////////////

  toolbar->addWidget(shapesCtl = new QShapesButton(this));
  shapesCtl->setSceneView(imageEditor->sceneView());


  /////////////////////

  toolbar->addAction(displaySettingsMenuAction = new QAction(getIcon(ICON_histogram), "Display options..."));
  displaySettingsMenuAction->setToolTip("Adjust display options");
  displaySettingsMenuAction->setCheckable(true);
  displaySettingsMenuAction->setChecked(false);
  connect(displaySettingsMenuAction, &QAction::triggered,
      this, &ThisClass::onDisplaySettingsMenuActionClicked);


  /////////////////////

  toolbar->addWidget(scaleSelectionCtl = new QScaleSelectionButton(this));
  scaleSelectionCtl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(scaleSelectionCtl, &QScaleSelectionButton::scaleChanged,
      [this](int v) {
        imageEditor->setViewScale(v);
      });


  /////////////////////
  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });

  connect(imageEditor, &QImageFileEditor::onMouseMove,
      [this, statusbar](QMouseEvent * e) {
        statusbar->showMessage(imageEditor->statusStringForPixel(e->pos()));
    });

  /////////////////////

  connect(imageEditor->scene(), &QImageScene::graphicsItemChanged,
      [this, statusbar](QGraphicsItem * item) {

        QGraphicsLineShape * lineShape = nullptr;
        QGraphicsRectShape * rectShape = nullptr;

        if ( (lineShape = dynamic_cast<QGraphicsLineShape * >(item)) ) {

          const QLineF line = lineShape->sceneLine();

          const QPointF p1 = line.p1();
          const QPointF p2 = line.p2();
          const double length = hypot(p2.x()-p1.x(), p2.y()-p1.y());
          const double angle = atan2(p2.y()-p1.y(), p2.x()-p1.x());

          statusbar->showMessage(qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
                  p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));

        }
        else if ( (rectShape = dynamic_cast<QGraphicsRectShape* >(item))) {

          CF_DEBUG("QImageScene::graphicsItemChanged");

          const QRectF rect = rectShape->mapToScene(rectShape->rect()).boundingRect();
          const QPointF p1 = rect.topLeft();
          const QPointF p2 = rect.bottomRight();
          const double width = rect.width();
          const double height = rect.height();

          statusbar->showMessage(qsprintf("p1: (%g %g)  p2: (%g %g)  %g x %g",
                  p1.x(), p1.y(), p2.x(), p2.y(), width, height));
        }
      });
}


void MainWindow::configureCloudViewerToolbars()
{
#if HAVE_QGLViewer
  QToolBar * toolbar;
  QAction * action;
  QLabel * imageNameLabel;
  QLabel * imageSizeLabel;
  QShortcut * shortcut;

  toolbar = cloudViewer->toolbar();

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
    QWaitCursor wait(this);
    cloudViewer->openPlyFile(cloudViewer->currentFileName());
  });


  toolbar->addSeparator();


  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  const auto onCloudViewerCurrentSourceChanged =
      [this, imageNameLabel] () {
        const QString abspath = cloudViewer->currentFileName();
        imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
      };

  connect(cloudViewer, &QCloudViewer::currentFileNameChanged,
      onCloudViewerCurrentSourceChanged);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());



  toolbar->addAction(action = new QAction(getIcon(ICON_options), "Options"));
  action->setToolTip("Configure cloud view options");
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, &QAction::triggered,
      [this, action](bool checked) {

        if ( checked && !cloudViewSettingsDialogBox ) {

          cloudViewSettingsDialogBox = new QCloudViewSettingsDialogBox(this);
          cloudViewSettingsDialogBox->setCloudViewer(cloudViewer);
          connect(cloudViewSettingsDialogBox, &QCloudViewSettingsDialogBox::visibilityChanged,
              action, &QAction::setChecked);
        }

        if ( cloudViewSettingsDialogBox ) {
          if ( !checked ) {
            cloudViewSettingsDialogBox->hide();
          }
          else {
            cloudViewSettingsDialogBox->setWindowTitle(QFileInfo(cloudViewer->currentFileName()).fileName());
            cloudViewSettingsDialogBox->showNormal();
          }
        }
      });


  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    cloudViewer->clear();
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });

#endif
}

void MainWindow::createDisplaySettingsControl()
{
  if( !mtfControl ) {

    mtfControl = new QMtfControlDialogBox(this);
    mtfControl->setMtfDisplaySettings(imageEditor->mtfDisplayFunction());

    connect(mtfControl, &QMtfControlDialogBox::visibilityChanged,
        [this](bool visible) {
          displaySettingsMenuAction->setChecked(visible);
        });
  }

}

void MainWindow::onDisplaySettingsMenuActionClicked(bool checked)
{
  if( !checked ) {
    if( mtfControl && mtfControl->isVisible() ) {
      mtfControl->hide();
    }
  }
  else {
    createDisplaySettingsControl();
    if( !mtfControl->isVisible() ) {
      mtfControl->show();
    }
  }
}

void MainWindow::createImageViewOptionsControl()
{
  if( !imageViewOptionsDlgBox ) {

    imageViewOptionsDlgBox = new QImageViewOptionsDlgBox(this);
    imageViewOptionsDlgBox->setImageViewer(imageEditor);

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

    imageViewOptionsDlgBox->show();
  }
}

void MainWindow::setupFocusGraph()
{
  focusMeasure_ =
      new QImageFocusMeasure(this);

  focusGraphDock_ =
      addDock<QFocusGraphDock>(this,
          Qt::RightDockWidgetArea,
          "focusGraphDock_",
          "Focus Graph",
          focusGraph_ = new QFocusGraph(this),
          viewMenu);

  focusGraphDock_->hide();
  focusGraph_->setFocusMeasureProvider(focusMeasure_);


  connect(imageEditor, &QImageFileEditor::onInputImageLoad,
      [this](const cv::Mat & image, const cv::Mat & mask, COLORID colorid, int bpp) {
        if ( focusMeasure_->enabled() ) {
          focusMeasure_->measure(image, colorid, bpp,
              imageEditor->roiRectShape()->isceneRect());
        }
      });

}

void MainWindow::setupRoiOptions()
{
  QAction * action;

  ///

  roiOptionsDialogBox_ =
      new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
          imageEditor->roiRectShape(),
          this);

  roiOptionsDialogBox_->loadParameters();

  roiActionsMenu_.addAction(action =
      createCheckableAction(QIcon(),
          "Options..",
          "Configure ROI rectangle options",
          [this](bool checked) {
            roiOptionsDialogBox_->setVisible(checked);
          }));

  connect(roiOptionsDialogBox_, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          imageEditor->roiRectShape()->setVisible(true);
        }
      });


  ///

  imageStatisticsDialogBox_ =
      new QImageStatisticsDisplayDialogBox("Measure...",
          this);

  imageStatisticsDialogBox_->display()->loadParameters();


  roiActionsMenu_.addAction(action =
      createCheckableAction(QIcon(ICON_metrics),
          "Measure..",
          "Measure image statistics in selected ROI",
          [this](bool checked) {
            imageStatisticsDialogBox_->setVisible(checked);
          }));

  connect(imageStatisticsDialogBox_, &QImageStatisticsDisplayDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          imageEditor->roiRectShape()->setVisible(true);
        }
      });

  ///

  connect(imageEditor->roiRectShape(), &QGraphicsShape::itemChanged,
      [this]() {

        QGraphicsRectShape * shape =
            imageEditor->roiRectShape();

        const QRectF rc =
            shape->sceneRect();

        imageEditor->statusbar()->showMessage(qsprintf(
                "ROI: x= %g y= %g w= %g h= %g center= (%g %g)",
                rc.x(), rc.y(), rc.width(), rc.height(),
                rc.center().x(), rc.center().y() ));
      });


  ///

  QToolBar * toolbar = imageEditor->embedToolbar();
  if ( toolbar ) {

    showRoiAction_ =
        createCheckableAction(getIcon(ICON_roi),
            "ROI Rectangle",
            "Show / Hide ROI rectangle",
            [this](bool checked) {
              imageEditor->roiRectShape()->setVisible(checked);
            });

    toolbar->addWidget(roiActionsButton_ =
        createToolButtonWithPopupMenu(showRoiAction_,
            &roiActionsMenu_));
  }

}

void MainWindow::openImage(const QString & abspath)
{
  QWaitCursor wait(this);

  if ( stackProgressView ) {
    stackProgressView->setImageViewer(nullptr);
  }

  imageEditor->clear();
  textViewer->clear();
#if HAVE_QGLViewer
  cloudViewer->clear();
#endif

  const QString suffix =
      QFileInfo(abspath).suffix();

  if ( isTextFileSuffix(suffix) ) {
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
  }
  else if ( isPlyFileSuffix(suffix) ) {
#if HAVE_QGLViewer
    if ( cloudViewer->openPlyFile(abspath) ) {
      centralStackedWidget->setCurrentWidget(cloudViewer);
    }
    else {
      centralStackedWidget->setCurrentWidget(textViewer);
      textViewer->showTextFile(abspath);
    }
#else
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
#endif
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
  if ( stackProgressView ) {

    // it may be that there is next task in queue,
    // don't blink with this dialog box
    QTimer::singleShot(500,
        [this]() {

          if ( !QStackingThread::isRunning() ) {
            if ( stackProgressView->isVisible() ) {
              stackProgressView->hide();
            }
            if ( stackOptionsView->isVisible() ) {
              stackOptionsView->setEnabled(true);
            }
          }
        });
  }
}

void MainWindow::onSaveCurrentImageAs()
{
  if ( !imageEditor->isVisible() || imageEditor->currentImage().empty() ) {
    return;
  }

  QString savedFileName = saveImageFileAs(this,
      imageEditor->currentImage(),
      imageEditor->currentMask(),
      imageEditor->current_processor(),
      imageEditor->currentFileName());

  if ( !savedFileName.isEmpty() ) {
    imageEditor->setCurrentFileName(savedFileName);
  }
}

void MainWindow::onSaveCurrentDisplayImageAs()
{
  if( imageEditor->isVisible() && !imageEditor->displayImage().empty() ) {

    saveImageFileAs(this,
        imageEditor->displayImage(),
        cv::Mat(),
        nullptr,
        imageEditor->currentFileName());
  }
}

void MainWindow::onSaveCurrentImageMask()
{
  if( imageEditor->isVisible() && !imageEditor->currentMask().empty() ) {

    saveImageFileAs(this,
        imageEditor->currentMask(),
        cv::Mat(),
        nullptr,
        QString("%1.mask.png").arg(imageEditor->currentFileName()));
  }
}

void MainWindow::onLoadCurrentImageMask()
{
  if( imageEditor->isVisible() && !imageEditor->currentImage().empty() ) {

    static const QString keyName = "lastImageMask";

    QSettings settings;

    static const QString filter =
        "Image files *.tiff *.tif *.png *.jpg (*.tiff *.tif *.png *.jpg);;\n"
        "All files (*);;";

    QString fileName =
        QFileDialog::getOpenFileName(this,
            "Select binary image", settings.value(keyName).toString(),
            filter,
            nullptr);

    if ( fileName.isEmpty() ) {
      return;
    }

    settings.setValue(keyName, fileName);

    cv::Mat image;

    if ( !load_image(fileName.toStdString(), image) ) {
      QMessageBox::critical(this, "Error",
          "load_image() fails.\n"
          "Can not load image from specified file");
      return;
    }

    if( image.type() != CV_8UC1 || image.size() != imageEditor->currentImage().size() ) {
      QMessageBox::critical(this, "Error",
          QString("Not appropriate mask image: %1x%2 depth=%3 channels=%4.\n"
              "Must be CV_8UC1 of the same size as current image (%5x%6)")
              .arg(image.cols)
              .arg(image.rows)
              .arg(image.depth())
              .arg(image.channels())
              .arg(imageEditor->currentImage().cols)
              .arg(imageEditor->currentImage().rows));
      return;
    }

    imageEditor->setMask(image, false);

  }
}


void MainWindow::saveCurrentWork()
{
  stacklist_->save();
}

void MainWindow::onLoadStackConfig()
{
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

    c_image_stacking_options::ptr stack =
        c_image_stacking_options::load(selectedFileNames[i].toStdString());

    if ( !stack ) {

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


    int pos = stacklist_->indexof(stack->name());
    if ( pos < 0 ) {
      stacklist_->add(stack);
      hasChanges = true;
    }
    else {

      const int responce =
          QMessageBox::critical(this, "ERROR",
              QString("Stack with name '%1' already exists.\n"
                  "Replace existing ?").arg(stack->cname()),
              QMessageBox::Yes | QMessageBox::No);

      if ( responce == QMessageBox::Yes  ) {
        stacklist_->set(pos, stack);
        hasChanges = true;
      }
    }
  }

  if ( hasChanges ) {
    stackTreeView->refresh();
  }
}

void MainWindow::onViewGeneralSettings()
{
  if ( !appSettingsDlgBox ) {
    appSettingsDlgBox = new QGeneralAppSettingsDialogBox(this);
  }

  if ( !appSettingsDlgBox->isVisible() ){
    appSettingsDlgBox->show();
  }
  else {
    appSettingsDlgBox->hide();
  }

  viewGeneralSettingsAction->setChecked(
      appSettingsDlgBox->isVisible());

}


///////////////////////////////////////////////////////////////////////////////
}  // namespace qskystacker
