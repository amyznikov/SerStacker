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

namespace serstacker {
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

#define ICON_roi          ":/serstacker/icons/roi.png"

MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/serstacker/icons/app-icon.png"));
  updateWindowTittle();
  QImageProcessorsCollection::load();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));
  centralStackedWidget->addWidget(imageEditor = new QImageEditor(this));
  centralStackedWidget->addWidget(textViewer = new QTextFileViewer(this));
  centralStackedWidget->addWidget(pipelineOptionsView = new QPipelineOptionsView(this));
  centralStackedWidget->addWidget(cloudViewer = new QCloudViewer(this));

  connect(centralStackedWidget, &QStackedWidget::currentChanged,
      [this]() {

        if ( is_visible(cloudViewSettingsDialogBox) ) {
          if ( !cloudViewer->isVisible() ) {
            cloudViewSettingsDialogBox->hide();
          }
        }

        if ( is_visible(mtfControl_) ) { // force update MTF histogram
          onMtfControlVisibilityChanged(true);
        }

      });



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
  setupImageEditor();
  setupTextViewer();
  stupCloudViewer();
  setupMeasures();
  setupRoiOptions();
  setupDisplayImageVideoWriter();
  setupProfileGraph();



  tabifyDockWidget(fileSystemTreeDock, sequencesTreeDock);
  //tabifyDockWidget(sequencesTreeDock, imageProcessorSelectorDock);
  tabifyDockWidget(sequencesTreeDock, imageProcessorDock_);


  connect(QPipelineThread::instance(), &QPipelineThread::started,
      this, &ThisClass::onPipelineThreadStarted);

  connect(QPipelineThread::instance(), &QPipelineThread::finished,
      this, &ThisClass::onPipelineThreadFinished);


  restoreState();

  imageEditor->set_current_processor(imageProcessor_ctl->current_processor());
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
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->displayPath(settings.value(
        "fileSystemTree/absoluteFilePath").toString());
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
            if ( w == imageEditor ) {
              imageEditor->openImage(imageEditor->currentFileName());
            }
            else if ( w == textViewer ) {
              textViewer->showTextFile(textViewer->currentFileName());
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

  saveImageAsAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentImage().empty());

  saveDisplayImageAsAction =
      menuFile_->addAction("Save current display image as...",
          this, &ThisClass::onSaveCurrentDisplayImageAs);

  saveDisplayImageAsAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->displayImage().empty());

  saveImageMaskAction =
      menuFile_->addAction("Save current image mask...",
          this, &ThisClass::onSaveCurrentImageMask);

  saveImageMaskAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentMask().empty());

  loadImageMaskAction =
      menuFile_->addAction("Set current image mask from file...",
          this, &ThisClass::onLoadCurrentImageMask);

  loadImageMaskAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentImage().empty());

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

  sb->addWidget(shapesLabel_ctl = new QLabel(this));
  sb->addWidget(mousePosLabel_ctl = new QLabel(this));

  sb->addPermanentWidget(showLog_ctl = new QToolButton());
  showLog_ctl->setDefaultAction(showLogWidgetAction_);
}

void MainWindow::setupDisplayImageVideoWriter()
{
  diplayImageWriter_.loadParameters();

  QToolBar *toolbar = imageEditor->toolbar();
  if( toolbar ) {

    toolbar->insertWidget(closeImageViewAction, displayImageVideoWriterToolButton_ =
        createDisplayVideoWriterOptionsToolButton(&diplayImageWriter_, this));
  }

}


void MainWindow::onStackProgressViewTextChanged()
{
  // FIXME : this is ugly temporary hotfix

  const QSize hint = pipelineProgressView->sizeHint();
  const QSize size = pipelineProgressView->size();

  // CF_DEBUG("sizeHint: %dx%d size=%dx%d", hint.width(), hint.height(), size.width(), size.height());
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
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if ( cloudViewer ) {
          cloudViewer->clear();
        }

        centralStackedWidget->setCurrentWidget(thumbnailsView);
        thumbnailsView->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::directoryItemPressed,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if ( cloudViewer ) {
          cloudViewer->clear();
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
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if (cloudViewer) {
          cloudViewer->clear();
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
  sequencesTreeDock =
      addImageSequenceTreeDock(this,
          Qt::LeftDockWidgetArea,
          "sequencesTreeDock",
          "Sequences",
          menuView_);

  sequencesTreeView =
      sequencesTreeDock->treeView();

  CF_DEBUG("sequencesTreeView->loadSequences()");
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
          centralStackedWidget->setCurrentWidget(imageEditor);
          pipelineProgressView->setImageViewer(imageEditor);
        }
      });

  connect(pipelineOptionsView, &QPipelineOptionsView::parameterChanged,
      this, &ThisClass::saveCurrentWork);

  connect(pipelineOptionsView, &QPipelineOptionsView::cloneCurrentPipelineRequested,
      [this]() {

        QWaitCursor wait (this);

        std::vector<c_image_sequence::sptr> selected_sequences;

        sequencesTreeView->getSelectedSequences(&selected_sequences);

        if ( !selected_sequences.empty() ) {
          if ( pipelineOptionsView->cloneCurrentPipeline(selected_sequences) ) {
            saveCurrentWork();
          }
        }
      });

}

void MainWindow::setupImageEditor()
{
  QToolBar *toolbar;

  toolbar = imageEditor->embedToolbar();

  imageEditor->addAction(copyDisplayImageAction =
      createAction(QIcon(),
          "Copy display image to clipboard (Ctrl+c)",
          "Copy display image to clipboard (Ctrl+c)",
          [this]() {
            if ( imageEditor->roiShape()->isVisible() ) {
              imageEditor->copyDisplayImageROIToClipboard(imageEditor->roiShape()->iSceneRect());
            }
            else {
              imageEditor->copyDisplayImageToClipboard();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuEdit_->addAction(copyDisplayImageAction);


  imageEditor->addAction(copyDisplayViewportAction =
      createAction(QIcon(),
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          "Copy display viewport to clipboard (Ctrl+SHIFT+C)",
          [this]() {
            if ( imageEditor->isVisible() ) {
              QPixmap pxmap = imageEditor->sceneView()->grab();
              if ( !pxmap.isNull() ) {
                QApplication::clipboard()->setPixmap(pxmap);
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C),
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuEdit_->addAction(copyDisplayViewportAction);


  connect(imageEditor, &QImageViewer::visibilityChanged,
      this, &ThisClass::onImageEditorVisibilityChanged);

  connect(imageEditor, &QImageViewer::currentFileNameChanged,
      this, &ThisClass::onImageEditorCurrentFileNameChanged);

  connect(imageEditor, &QImageEditor::currentImageChanged,
      this, &ThisClass::onImageEditorCurrentImageChanged);

  connect(imageEditor, &QImageEditor::displayImageChanged,
      this, &ThisClass::onImageEditorDisplayImageChanged);


  ///
  /// Configure image editor toolbar
  ///

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
        thumbnailsView->moveToBads(imageEditor->currentFileName());
      },
      new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Delete),
          imageEditor, nullptr, nullptr,
          Qt::WindowShortcut)));

  toolbar->addAction(badframeAction =
      createCheckableAction(badframeIcon,
          "Bad Frame",
          "Mark / Unmark current frame as bad (Ctrl+A)",
          false,
          [this](bool checked) {
            if ( imageEditor->isVisible() ) {

              const c_input_sequence::sptr & input_sequence = imageEditor->input_sequence();
              if ( input_sequence ) {

                c_input_source::sptr source = input_sequence->current_source();
                if ( source ) {
                  source->set_badframe(input_sequence->current_pos() - 1, checked);
                  source->save_badframes();
                }
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_A),
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  toolbar->addAction(setReferenceFrameAction =
      createAction(getIcon(ICON_reference),
          "Make reference",
          "Make this frame reference",
          [this]() {
            if ( imageEditor->isVisible() && sequencesTreeView->isVisible() ) {

              c_input_source::sptr selected_source;
              c_image_sequence::sptr selected_sequence;

              selected_source =
                  sequencesTreeView->getCurrentInputSource(
                      &selected_sequence);

              if ( selected_source && selected_sequence ) {

                const c_input_sequence::sptr & currentSequence =
                    imageEditor->input_sequence();

                if ( currentSequence && currentSequence->current_source()->filename() == selected_source->filename() ) {

                  const c_image_processing_pipeline::sptr pipeline =
                      selected_sequence->current_pipeline();

                  if ( pipeline && pipeline->has_master_frame() ) {
                    pipeline->set_master_source(selected_source->filename());
                    pipeline->set_master_frame_index(currentSequence->current_pos() - 1);
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

  toolbar->addWidget(imageNameLabel_ctl = new QLabel(""));
  imageNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);

  toolbar->addSeparator();

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
            if ( imageEditor->isVisible() ) {

              if ( checked ) {
                setupImageViewOptions();
              }
              else if ( imageViewOptionsDlgBox ) {
                delete imageViewOptionsDlgBox;
                imageViewOptionsDlgBox = nullptr;
              }
            }
          }));

  toolbar->addWidget(shapes_ctl =
      new QShapesButton(imageEditor->sceneView(),
          this));

  toolbar->addAction(showMtfControlAction_);

  toolbar->addWidget(scaleSelection_ctl = new QScaleSelectionButton(this));
  scaleSelection_ctl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(scaleSelection_ctl, &QScaleSelectionButton::scaleChanged,
      [this](int v) {
        imageEditor->setViewScale(v);
      });


  toolbar->addAction(closeImageViewAction =
      createAction(getIcon(ICON_close),
          "Close",
          "Close window",
          [this]() {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          },
          new QShortcut(QKeySequence::Cancel,
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  connect(imageEditor, &QImageFileEditor::onMouseMove,
      [this](QMouseEvent * e) {
        mousePosLabel_ctl->setText(imageEditor->statusStringForPixel(e->pos()));
      });

  connect(imageEditor->scene(), &QImageScene::graphicsItemChanged,
      [this](QGraphicsItem * item) {

        QGraphicsLineShape * lineShape = nullptr;
        QGraphicsRectShape * rectShape = nullptr;

        if ( (lineShape = dynamic_cast<QGraphicsLineShape * >(item)) ) {

          const QLineF line = lineShape->sceneLine();

          const QPointF p1 = line.p1();
          const QPointF p2 = line.p2();
          const double length = hypot(p2.x()-p1.x(), p2.y()-p1.y());
          const double angle = atan2(p2.y()-p1.y(), p2.x()-p1.x());

          if ( !shapesLabel_ctl->isVisible() ) {
            shapesLabel_ctl->setVisible(true);
          }

          shapesLabel_ctl->setText(
              qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
                  p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));

          if ( is_visible(profileGraph_ctl_) ) {
            profileGraph_ctl_->showProfilePlot(line, imageEditor->currentImage());
          }

        }
        else if ( (rectShape = dynamic_cast<QGraphicsRectShape* >(item))) {

          const QRectF rc = rectShape->mapToScene(rectShape->rect()).boundingRect();

          const QPointF p1 = rc.topLeft();
          const QPointF p2 = rc.bottomRight();
          const QPointF center = rc.center();
          const double width = rc.width();
          const double height = rc.height();

          if ( !shapesLabel_ctl->isVisible() ) {
            shapesLabel_ctl->setVisible(true);
          }

          shapesLabel_ctl->setText(
              qsprintf("RECT: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                  p1.x(), p1.y(),
                  p2.x(), p2.y(),
                  width, height,
                  center.x(), center.y()));

        }
      });

  connect(imageEditor->scene(), &QImageScene::graphicsItemVisibleChanged,
      [this]() {
        if ( shapesLabel_ctl->isVisible() ) {
          shapesLabel_ctl->setVisible(false);
        }
      });

  connect(imageEditor->scene(), &QImageScene::graphicsItemDestroyed,
      [this]() {
        if ( shapesLabel_ctl->isVisible() ) {
          shapesLabel_ctl->setVisible(false);
        }
      });

}


void MainWindow::setupTextViewer()
{
  QToolBar *toolbar;
  QLabel *imageNameLabel;
  QLabel *imageSizeLabel;

  toolbar = textViewer->toolbar();

  toolbar->addAction(selectPreviousFileAction_);
  toolbar->addAction(selectNextFileAction);
  toolbar->addAction(reloadCurrentFileAction);

  toolbar->addSeparator();

  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());

  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          textViewer, nullptr, nullptr,
          Qt::WindowShortcut)));


  connect(textViewer, &QTextFileViewer::currentFileNameChanged,
      [this, imageNameLabel]() {
        const QString abspath = textViewer->currentFileName();
        imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
      });

  connect(textViewer, &QTextFileViewer::visibilityChanged,
      [this](bool visible) {
        if ( !visible ) {
          textViewer->clear();
        }
      });
}

void MainWindow::onImageEditorCurrentFileNameChanged()
{
  const QString abspath =
      imageEditor->currentFileName();

  imageNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
  //imageSizeLabel_ctl->setText(QString("%1x%2").arg(imageEditor->currentImage().cols).arg(imageEditor->currentImage().rows));

  if ( is_visible(mtfControl_) ) {

//    if( mtfControl_->mtfDisplaySettings() != imageEditor->mtfDisplayFunction() ) {
//      mtfControl_->setMtfDisplaySettings(imageEditor->mtfDisplayFunction());
//    }

    if ( imageEditor->currentFileName().isEmpty() ) {
      mtfControl_->setWindowTitle("Adjust Display Levels ...");
    }
    else {
      mtfControl_->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
    }
  }
}

void MainWindow::onImageEditorCheckIfBadFrameSelected()
{
  bool isBadFrameSelected = false;

  if( imageEditor->isVisible() ) {

    const c_input_sequence::sptr &input_sequence =
        imageEditor->input_sequence();

    if( input_sequence ) {

      c_input_source::sptr source =
          input_sequence->current_source();

      if( source ) {
        isBadFrameSelected =
            source->is_badframe(input_sequence->current_pos() - 1);
      }
    }
  }

  if( badframeAction->isChecked() != isBadFrameSelected ) {
    badframeAction->setChecked(isBadFrameSelected);
  }
}


void MainWindow::onImageEditorVisibilityChanged(bool isvisible)
{
  const bool hasimage =
      !imageEditor->currentImage().empty();

  const bool hasmask =
      !imageEditor->currentMask().empty();

  const bool hasdisplayimage =
      !imageEditor->displayImage().empty();

  saveImageAsAction->setEnabled(isvisible && hasimage);
  saveDisplayImageAsAction->setEnabled(isvisible && hasdisplayimage);
  copyDisplayImageAction->setEnabled(isvisible && hasdisplayimage);
  saveImageMaskAction->setEnabled(isvisible && hasmask);
  loadImageMaskAction->setEnabled(isvisible && hasimage);

  onImageEditorCheckIfBadFrameSelected();
}

void MainWindow::onImageEditorCurrentImageChanged()
{
  const bool isvisible =
      imageEditor->isVisible();

  const bool hasimage =
      !imageEditor->currentImage().empty();

  const bool hasmask =
      !imageEditor->currentMask().empty();

  saveImageAsAction->setEnabled(isvisible && hasimage);
  saveImageMaskAction->setEnabled(isvisible && hasmask);
  loadImageMaskAction->setEnabled(isvisible && hasimage);

  if ( isvisible ) {

    imageSizeLabel_ctl->setText(qsprintf("%dx%d",
        imageEditor->currentImage().cols,
        imageEditor->currentImage().rows));

    if( is_visible(profileGraph_ctl_) ) {
      profileGraph_ctl_->showProfilePlot(profileGraph_ctl_->currentLine(),
          imageEditor->currentImage());
    }

    updateMeasurements();

    onImageEditorCheckIfBadFrameSelected();
  }

  if ( is_visible(mtfControl_) ) {
    onMtfControlVisibilityChanged(true);
  }

}

void MainWindow::onImageEditorDisplayImageChanged()
{
  const bool isvisible =
      imageEditor->isVisible();

  const bool hasdisplayimage =
      !imageEditor->displayImage().empty();

  saveDisplayImageAsAction->setEnabled(isvisible && hasdisplayimage);
  copyDisplayImageAction->setEnabled(isvisible && hasdisplayimage);

  if ( diplayImageWriter_.started() ) {

    if ( !imageEditor->isVisible() ) {
      diplayImageWriter_.stop();
    }
    else if ( !imageEditor->displayImage().empty() ) {
      diplayImageWriter_.write(imageEditor->displayImage());
    }
  }

}


void MainWindow::onImageProcessorParameterChanged()
{
  Base::onImageProcessorParameterChanged();
  imageEditor->set_current_processor(imageProcessor_ctl->current_processor());
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  if( !visible ) {
    mtfControl_->setMtfDisplaySettings(nullptr);
  }
  else if( is_visible(imageEditor) ) {

    mtfControl_->setMtfDisplaySettings(imageEditor->mtfDisplayFunction());

    if ( imageEditor->currentFileName().isEmpty() ) {
      mtfControl_->setWindowTitle("Adjust Display Levels ...");
    }
    else {
      mtfControl_->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
    }
  }
  else if( is_visible(cloudViewer) ) {

    mtfControl_->setMtfDisplaySettings(&cloudViewer->mtfDisplay());

    if ( cloudViewer->currentFileName().isEmpty() ) {
      mtfControl_->setWindowTitle("Adjust Display Levels ...");
    }
    else {
      mtfControl_->setWindowTitle(QFileInfo(cloudViewer->currentFileName()).fileName());
    }
  }
  else {
    mtfControl_->setMtfDisplaySettings(nullptr);
  }

}


void MainWindow::stupCloudViewer()
{
  QToolBar * toolbar;
  //QToolButton * toolbutton;
  QAction * action;
  QLabel * imageNameLabel_ctl;
  QLabel * imageSizeLabel_ctl;
  QShortcut * shortcut;

  toolbar = cloudViewer->toolbar();

  toolbar->addAction(action = new QAction(getIcon(ICON_prev), "Previous"));
  action->setToolTip("Load previous image from list");
  //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectPrevIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_next), "Next"));
  action->setToolTip("Load next image from list");
  //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
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


  toolbar->addWidget(imageNameLabel_ctl = new QLabel(""));
  imageNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());

  toolbar->addAction(showMtfControlAction_);

  toolbar->addWidget(createToolButton(getIcon(ICON_options),
      "Options",
      "Cloud View Options...",
      [this](QToolButton * tb) {

        QMenu menu;

        menu.addAction(createMenuWidgetAction<QSpinBox>("Point size: ",
                nullptr,
                [this](const auto * action) {
                  QSpinBox * spinBox = action->control();
                  spinBox->setKeyboardTracking(false);
                  spinBox->setRange(1, 32);
                  spinBox->setValue((int)cloudViewer->pointSize());
                  connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                      [this](int value) {
                        cloudViewer->setPointSize(value);
                      });
                }));

        menu.addAction(createMenuWidgetAction<QSpinBox>("Point brightness: ",
                nullptr,
                [this](const auto * action) {
                  QSpinBox * spinBox = action->control();
                  spinBox->setKeyboardTracking(false);
                  spinBox->setRange(-128, 128);
                  spinBox->setValue((int)cloudViewer->pointBrightness());
                  connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                      [this](int value) {
                        cloudViewer->setPointBrightness(value);
                      });
                }));

        menu.addAction(createAction(QIcon(),
                "Show cloud center",
                "Rotate camera to show point cloud center",
                [this]() {
                  if ( cloudViewer ) {
                    cloudViewer->rotateToShowCloud();
                  }
                }));

        menu.addAction(createCheckableAction(QIcon(),
                "Auto Show Target point",
                "",
                cloudViewer->autoShowViewTarget(),
                [this](bool checked) {
                  if ( cloudViewer ) {
                    cloudViewer->setAutoShowViewTarget(checked);
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


  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });

  connect(cloudViewer, &QCloudViewer::currentFileNameChanged,
      [this, imageNameLabel_ctl]() {
        const QString abspath = cloudViewer->currentFileName();
        imageNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
        if ( is_visible(mtfControl_) ) {
          onMtfControlVisibilityChanged(true);
        }
      });

  connect(cloudViewer, &QCloudViewer::visibilityChanged,
      [this](bool visible) {
        if ( !visible ) {
          cloudViewer->clear();
        }
      });

}

void MainWindow::onShowCloudViewSettingsDialogBoxActionClicked(bool checked)
{
  if ( checked && !cloudViewSettingsDialogBox ) {

    cloudViewSettingsDialogBox = new QCloudViewSettingsDialogBox(this);
    cloudViewSettingsDialogBox->setCloudViewer(cloudViewer);
    connect(cloudViewSettingsDialogBox, &QCloudViewSettingsDialogBox::visibilityChanged,
        showCloudViewSettingsDialogBoxAction, &QAction::setChecked);
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

}

void MainWindow::updateMeasurements()
{
  if( !QMeasureProvider::requested_measures().empty() && imageEditor->roiShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(imageEditor);

    if ( !imageEditor->currentImage().empty() ) {

      QMeasureProvider::compute(imageEditor->currentImage(),
          imageEditor->currentMask(),
          imageEditor->roiShape()->iSceneRect());

    }
  }
}

void MainWindow::onMeasureRightNowRequested()
{
  CF_DEBUG("MainWindow::MeasureRequested");
  updateMeasurements();
}

void MainWindow::setupRoiOptions()
{
  QAction * action;

  ///

  roiOptionsDialogBox_ =
      new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
          imageEditor->roiShape(),
          this);

  roiOptionsDialogBox_->loadParameters();

  roiActionsMenu_.addAction(action =
      createCheckableAction(QIcon(),
          "ROI Options..",
          "Configure ROI rectangle options",
          is_visible(roiOptionsDialogBox_),
          [this](bool checked) {
            roiOptionsDialogBox_->setVisible(checked);
          }));

  connect(roiOptionsDialogBox_, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          imageEditor->roiShape()->setVisible(true);
        }
      });

  roiActionsMenu_.addAction(showMeasuresSettingsAction_);
  roiActionsMenu_.addAction(showMeasuresDisplayAction_);
  roiActionsMenu_.addAction(showMeasuresGraphAction_);

  ///

  connect(imageEditor->roiShape(), &QGraphicsShape::itemChanged,
      [this]() {

        QGraphicsRectShape * shape =
            imageEditor->roiShape();

        const QRectF rc =
            shape->sceneRect();

        const QPointF p1 = rc.topLeft();
        const QPointF p2 = rc.bottomRight();
        const QPointF center = rc.center();
        const double width = rc.width();
        const double height = rc.height();

        if ( !shapesLabel_ctl->isVisible() ) {
          shapesLabel_ctl->setVisible(true);
        }

        shapesLabel_ctl->setText(
            qsprintf("ROI: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                p1.x(), p1.y(),
                p2.x(), p2.y(),
                width, height,
                center.x(), center.y()));

        //CF_DEBUG("C updateMeasurements()");
        updateMeasurements();
      });

  connect(imageEditor->roiShape(), &QGraphicsShape::visibleChanged,
      [this]() {
        if ( shapesLabel_ctl->isVisible() ) {
          shapesLabel_ctl->setVisible(false);
        }
      });


  ///

  QToolBar * toolbar = imageEditor->toolbar();
  if ( toolbar ) {

    showRoiAction_ =
        createCheckableAction(getIcon(ICON_roi),
            "ROI Rectangle",
            "Show / Hide ROI rectangle",
            is_visible(imageEditor) && imageEditor->roiShape()->isVisible(),
            [this](bool checked) {
              imageEditor->roiShape()->setVisible(checked);
            });

    toolbar->insertWidget(closeImageViewAction,
        roiActionsButton_ =
            createToolButtonWithPopupMenu(showRoiAction_,
                &roiActionsMenu_));

    connect(imageEditor->roiShape(), &QGraphicsObject::visibleChanged,
        [this]() {
          showRoiAction_->setChecked(imageEditor->roiShape()->isVisible());
        });

    showRoiAction_->setChecked(imageEditor->roiShape()->isVisible());
  }

}

void MainWindow::setupImageViewOptions()
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


void MainWindow::openImage(const QString & abspath)
{
  QWaitCursor wait(this);

  if ( pipelineProgressView ) {
    pipelineProgressView->setImageViewer(nullptr);
  }

  const QString suffix =
      QFileInfo(abspath).suffix();

  if ( isTextFileSuffix(suffix) ) {
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
  }
  else if ( isPlyFileSuffix(suffix) ) {
    if ( cloudViewer->openPlyFile(abspath) ) {
      centralStackedWidget->setCurrentWidget(cloudViewer);
    }
    else {
      centralStackedWidget->setCurrentWidget(textViewer);
      textViewer->showTextFile(abspath);
    }
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


  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->fillContextMenu(menu, selectedItems);
  }

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
    centralStackedWidget->setCurrentWidget(imageEditor);
    imageEditor->openImage(source->filename());
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
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(source->filename());
    }
    else if ( sequence ) {

      QWidget * currentCentralWidget =
          centralStackedWidget->currentWidget();

      const c_image_processing_pipeline::sptr & pipeline =
          QPipelineThread::currentPipeline();

      if ( pipeline && sequence == pipeline->input_sequence() ) {

        if ( currentCentralWidget == imageEditor ) {

          if ( !pipelineProgressView->imageViewer() ) {
            pipelineProgressView->setImageViewer(imageEditor);
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
            centralStackedWidget->setCurrentWidget(imageEditor);
            pipelineProgressView->setImageViewer(imageEditor);
          }
        }
        else {
          centralStackedWidget->setCurrentWidget(imageEditor);
          pipelineProgressView->setImageViewer(imageEditor);
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
    if ( centralStackedWidget->currentWidget() == imageEditor ) {
      centralStackedWidget->setCurrentWidget(thumbnailsView);
      thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
    }
    else {
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(source->filename());
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
  if ( centralStackedWidget->currentWidget() != imageEditor ) {
    centralStackedWidget->setCurrentWidget(imageEditor);
  }

  imageEditor->clear();
  pipelineProgressView->setImageViewer(imageEditor);

  if ( !pipelineProgressView->isVisible() ) {
    pipelineProgressView->show();
  }
}

void MainWindow::onPipelineThreadFinished()
{
  if ( pipelineProgressView ) {

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
    inputOptionsDlgBox->setImageEditor(imageEditor);
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


///////////////////////////////////////////////////////////////////////////////
}  // namespace qskystacker
